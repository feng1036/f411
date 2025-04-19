#include <string.h>
#include "sys.h"
#include "config.h"
#include "uart_syslink.h"
#include "debug_assert.h"
#include "config.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"



#define UARTSLK_DATA_TIMEOUT_MS 	1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((u32)0x00000001)

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

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* 使能GPIO 和 UART 时钟*/
	RCC_AHB1PeriphClockCmd(UARTSLK_GPIO_PERIF, ENABLE);
	ENABLE_UARTSLK_RCC(UARTSLK_PERIF, ENABLE);

	/* 配置USART Rx为浮空输入*/
	GPIO_InitStructure.GPIO_Pin   = UARTSLK_GPIO_RX_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UARTSLK_GPIO_PORT, &GPIO_InitStructure);

	/* 配置USART Tx 复用功能输出*/
	GPIO_InitStructure.GPIO_Pin   = UARTSLK_GPIO_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_Init(UARTSLK_GPIO_PORT, &GPIO_InitStructure);

	/*端口映射*/
	GPIO_PinAFConfig(UARTSLK_GPIO_PORT, UARTSLK_GPIO_AF_TX_PIN, UARTSLK_GPIO_AF_TX);
	GPIO_PinAFConfig(UARTSLK_GPIO_PORT, UARTSLK_GPIO_AF_RX_PIN, UARTSLK_GPIO_AF_RX);


	USART_InitStructure.USART_BaudRate            = 1000000;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UARTSLK_TYPE, &USART_InitStructure);


	/*配置串口非空中断*/
	NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/*串口接收数据寄存器非空中断*/
	USART_ITConfig(UARTSLK_TYPE, USART_IT_RXNE, ENABLE);	

	/*配置TXEN引脚(NRF流控制)*/
	RCC_AHB1PeriphClockCmd(UARTSLK_TXEN_PERIF, ENABLE);

	memset(&GPIO_InitStructure, 0,sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin = UARTSLK_TXEN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UARTSLK_TXEN_PORT, &GPIO_InitStructure);

	/*PA0外部中断配置*/
	EXTI_InitStructure.EXTI_Line = UARTSLK_TXEN_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(UARTSLK_TXEN_EXTI);

	NVIC_EnableIRQ(EXTI0_IRQn);

	USART_Cmd(UARTSLK_TYPE, ENABLE);	/*使能定时器*/
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
		while (!(UARTSLK_TYPE->SR & USART_FLAG_TXE))
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
	USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, ENABLE);	/*串口发送数据寄存器为空中断*/
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
	else if (USART_GetITStatus(UARTSLK_TYPE, USART_IT_TXE) == SET)
	{
		if (outDataIsr && (dataIndexIsr < dataSizeIsr))
		{
			USART_SendData(UARTSLK_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
			dataIndexIsr++;
		} else
		{
			USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, DISABLE);
			xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
		}
	}
}

// void uartslkTxenFlowctrlIsr()
// {
// 	EXTI_ClearFlag(UARTSLK_TXEN_EXTI);
// 	if (GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET)
// 	{
// 		uartslkPauseDma();
// 		//ledSet(LED_GREEN_R, 1);
// 	}
// 	else
// 	{
// 		uartslkResumeDma();
// 		//ledSet(LED_GREEN_R, 0);
// 	}
// }

// void __attribute__((used)) EXTI0_Callback(void)
// {
// 	uartslkTxenFlowctrlIsr();
// }

void __attribute__((used)) USART2_IRQHandler(void)	
{
	uartslkIsr();
}

// void __attribute__((used)) DMA1_Stream6_IRQHandler(void)	
// {
// 	uartslkDmaIsr();
// }

