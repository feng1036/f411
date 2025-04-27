#include <string.h>
#include "sys.h"
#include "uart_syslink.h"
#include "debug_assert.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define GPIO_Pin_0                 ((uint16_t)0x0001)
#define GPIO_Pin_2                 ((uint16_t)0x0004)
#define GPIO_Pin_3                 ((uint16_t)0x0008)
#define CR1_CLEAR_MASK            ((uint16_t)0x160C)
#define CR3_CLEAR_MASK            ((uint16_t)0x0300)

#define UARTSLK_DATA_TIMEOUT_MS 	1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((uint_32)0x00000001)

static bool isInit = false;

static xSemaphoreHandle waitUntilSendDone;
static xSemaphoreHandle uartBusy;
static xQueueHandle uartslkDataDelivery;

static u8 *outDataIsr;
static u8 dataIndexIsr;
static u8 dataSizeIsr;

void uartslkInit(void)	/*串口初始化*/
{
	waitUntilSendDone = xSemaphoreCreateBinary(); 	/*等待发送完成 二值信号量*/
	uartBusy = xSemaphoreCreateBinary();			/*串口忙 二值信号量*/
	xSemaphoreGive(uartBusy); 
	
	uartslkDataDelivery = xQueueCreate(1024, sizeof(u8));	/*队列 1024个消息*/
	ASSERT(uartslkDataDelivery);

	/* 使能GPIO 和 UART 时钟*/
	RCC->AHB1ENR |= (uint32_t)0x00000001;
	RCC->APB1ENR |= (uint32_t)0x00020000;

	/* 配置USART Rx为浮空输入*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER3);
	GPIOA->MODER |= (uint32_t)(0x80);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3);
	GPIOA->OTYPER  &= ~((GPIO_OTYPER_OT_3));
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	GPIOA->PUPDR |= (uint32_t)(0x40);

	// /* 配置USART Tx 复用功能输出*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);
	GPIOA->MODER |= (uint32_t)(0x20);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR2);
    GPIOA->OSPEEDR |= (uint32_t)(0x10);
	GPIOA->OTYPER  &= ~((GPIO_OTYPER_OT_2));
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	GPIOA->PUPDR |= (uint32_t)(0x10);

	/*端口映射*/
	GPIOA->AFR[0]&=~((uint32_t)0xF<<(8U));
	GPIOA->AFR[0]|=((uint32_t)(0x07)<<(8U));
	GPIOA->AFR[0]&=~((uint32_t)0xF<<(12U));
	GPIOA->AFR[0]|=((uint32_t)(0x7)<<(12U));
	
	USART2->CR2 &= ~USART_CR2_STOP;
	USART2->CR1 &= ~CR1_CLEAR_MASK;
	USART2->CR1 |=0x000C;
	USART2->CR3 &=~CR3_CLEAR_MASK;
	USART2->BRR = (uint16_t)48;

	// /*配置串口非空中断*/
    NVIC->IP[USART2_IRQn]=(uint32_t)(((5U)&0x0F)<<4);
    NVIC->ISER[USART2_IRQn>>0x05]=((uint32_t)1U<<(USART2_IRQn&(uint8_t)0x1F));
	/*串口接收数据寄存器非空中断*/
	USART2->CR1|=(uint32_t)(0x20);
	/*配置TXEN引脚(NRF流控制)*/
	RCC->AHB1ENR|=(uint32_t)0x00000001;
	GPIOA->MODER  &= ~(GPIO_MODER_MODER0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
	GPIOA->PUPDR |= (uint32_t)0x04;

	EXTI->IMR&=~(uint32_t)0x00001;
    EXTI->EMR&=~(uint32_t)0x00001;
	EXTI->IMR |=(uint32_t)0x00001;
	EXTI->RTSR&=~(uint32_t)0x00001;
    EXTI->FTSR&=~(uint32_t)0x00001;
	EXTI->PR = (uint32_t)0x00001;

	NVIC->ISER[(uint32_t)((int32_t)EXTI0_IRQn) >> 5] = (uint32_t)(1 << ((uint32_t)((int32_t)EXTI0_IRQn) & (uint32_t)0x1F));

	USART2->CR1 |=(uint16_t)0x2000;
	isInit = true;
}

bool uartslkTest(void)
{
	return isInit;
}
/*从接收队列读取数据(带超时处理)*/
bool uartslkGetDataWithTimout(u8 *c)
{
	/*接收uartslkDataDelivery(1024个容量)消息*/
	if (xQueueReceive(uartslkDataDelivery, c, UARTSLK_DATA_TIMEOUT_TICKS) == pdTRUE)	
	{
		return true;
	}
	*c = 0;
	return false;
}
/*发送原始数据*/
void uartslkSendData(u32 size, u8* data)
{
	u32 i;
	if (!isInit) return;
	for(i = 0; i < size; i++)
	{
	#ifdef UARTSLK_SPINLOOP_FLOWCTRL
		while(GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET);
	#endif
		while (!(UARTSLK_TYPE->SR & (uint16_t)0x0080))
		{};
		UARTSLK_TYPE->DR = (data[i] & 0x00FF);
	}
}
/*中断方式发送原始数据*/
void uartslkSendDataIsrBlocking(u32 size, u8* data)
{
	xSemaphoreTake(uartBusy, portMAX_DELAY);
	outDataIsr = data;
	dataSizeIsr = size;
	dataIndexIsr = 1;
	uartslkSendData(1, &data[0]);
	USART2->CR1|=(uint32_t)(0x80);
	xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
	outDataIsr = 0;
	xSemaphoreGive(uartBusy);
}
/*发送一个字符到串口*/
int uartslkPutchar(int ch)
{
    uartslkSendData(1, (u8 *)&ch);
    return (u8)ch;
}

void uartslkIsr(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if ((UARTSLK_TYPE->SR & (1<<5)) != 0) /*接收非空中断*/
	{
		u8 rxDataInterrupt = (u8)(UARTSLK_TYPE->DR & 0xFF);
		xQueueSendFromISR(uartslkDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
	}
	else if ((USART2->CR1&0x80)!=RESET&&(USART2->SR&0x80)!=RESET)
	{
		if (outDataIsr && (dataIndexIsr < dataSizeIsr))
		{
			USART2->DR = ((outDataIsr[dataIndexIsr] & 0x00FF) & (uint16_t)0x01FF);
			dataIndexIsr++;
		} else
		{
			USART2->CR1|=(uint32_t)(0x80);
			xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
		}
	}
}


void __attribute__((used)) USART2_IRQHandler(void)	
{
	uartslkIsr();
}
