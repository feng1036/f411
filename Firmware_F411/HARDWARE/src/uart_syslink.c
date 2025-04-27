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

void uartslkInit(void)	/*���ڳ�ʼ��*/
{
	waitUntilSendDone = xSemaphoreCreateBinary(); 	/*�ȴ�������� ��ֵ�ź���*/
	uartBusy = xSemaphoreCreateBinary();			/*����æ ��ֵ�ź���*/
	xSemaphoreGive(uartBusy); 
	
	uartslkDataDelivery = xQueueCreate(1024, sizeof(u8));	/*���� 1024����Ϣ*/
	ASSERT(uartslkDataDelivery);

	/* ʹ��GPIO �� UART ʱ��*/
	RCC->AHB1ENR |= (uint32_t)0x00000001;
	RCC->APB1ENR |= (uint32_t)0x00020000;

	/* ����USART RxΪ��������*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER3);
	GPIOA->MODER |= (uint32_t)(0x80);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3);
	GPIOA->OTYPER  &= ~((GPIO_OTYPER_OT_3));
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	GPIOA->PUPDR |= (uint32_t)(0x40);

	// /* ����USART Tx ���ù������*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);
	GPIOA->MODER |= (uint32_t)(0x20);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR2);
    GPIOA->OSPEEDR |= (uint32_t)(0x10);
	GPIOA->OTYPER  &= ~((GPIO_OTYPER_OT_2));
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	GPIOA->PUPDR |= (uint32_t)(0x10);

	/*�˿�ӳ��*/
	GPIOA->AFR[0]&=~((uint32_t)0xF<<(8U));
	GPIOA->AFR[0]|=((uint32_t)(0x07)<<(8U));
	GPIOA->AFR[0]&=~((uint32_t)0xF<<(12U));
	GPIOA->AFR[0]|=((uint32_t)(0x7)<<(12U));
	
	USART2->CR2 &= ~USART_CR2_STOP;
	USART2->CR1 &= ~CR1_CLEAR_MASK;
	USART2->CR1 |=0x000C;
	USART2->CR3 &=~CR3_CLEAR_MASK;
	USART2->BRR = (uint16_t)48;

	// /*���ô��ڷǿ��ж�*/
    NVIC->IP[USART2_IRQn]=(uint32_t)(((5U)&0x0F)<<4);
    NVIC->ISER[USART2_IRQn>>0x05]=((uint32_t)1U<<(USART2_IRQn&(uint8_t)0x1F));
	/*���ڽ������ݼĴ����ǿ��ж�*/
	USART2->CR1|=(uint32_t)(0x20);
	/*����TXEN����(NRF������)*/
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
/*�ӽ��ն��ж�ȡ����(����ʱ����)*/
bool uartslkGetDataWithTimout(u8 *c)
{
	/*����uartslkDataDelivery(1024������)��Ϣ*/
	if (xQueueReceive(uartslkDataDelivery, c, UARTSLK_DATA_TIMEOUT_TICKS) == pdTRUE)	
	{
		return true;
	}
	*c = 0;
	return false;
}
/*����ԭʼ����*/
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
/*�жϷ�ʽ����ԭʼ����*/
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
/*����һ���ַ�������*/
int uartslkPutchar(int ch)
{
    uartslkSendData(1, (u8 *)&ch);
    return (u8)ch;
}

void uartslkIsr(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if ((UARTSLK_TYPE->SR & (1<<5)) != 0) /*���շǿ��ж�*/
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
