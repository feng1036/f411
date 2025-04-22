#include <string.h>
#include "sys.h"
#include "config.h"
#include "uart_syslink.h"
#include "debug_assert.h"
// #include "stm32f4xx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define UARTSLK_DATA_TIMEOUT_MS 	1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((u32)0x00000001)

// USART2相关宏定义(使用标准寄存器定义更为安全)
#define UARTSLK_SR_RXNE        USART_SR_RXNE
#define UARTSLK_SR_TXE         USART_SR_TXE 

// EXTI配置用的端口源定义
// 这里需要根据UARTSLK_TXEN_PORT实际使用的端口来设置
// GPIOA=0, GPIOB=1, GPIOC=2, 依此类推
#define UARTSLK_TXEN_PORT_SOURCE    0  // 假设TXEN引脚在GPIOA上，根据实际情况调整

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

	// 使能GPIO和USART时钟
	RCC->AHB1ENR |= UARTSLK_GPIO_PERIF;  // 使能GPIO时钟
	RCC->APB1ENR |= UARTSLK_PERIF;       // 使能USART时钟(假设是APB1上的外设)

	// 配置GPIO管脚
	// 清除相关位
	UARTSLK_GPIO_PORT->MODER &= ~(0x3 << (UARTSLK_GPIO_AF_RX_PIN * 2));
	UARTSLK_GPIO_PORT->MODER |= (0x2 << (UARTSLK_GPIO_AF_RX_PIN * 2));  // 设置为复用功能
	UARTSLK_GPIO_PORT->PUPDR &= ~(0x3 << (UARTSLK_GPIO_AF_RX_PIN * 2));
	UARTSLK_GPIO_PORT->PUPDR |= (0x1 << (UARTSLK_GPIO_AF_RX_PIN * 2));  // 上拉

	// 配置TX引脚
	UARTSLK_GPIO_PORT->MODER &= ~(0x3 << (UARTSLK_GPIO_AF_TX_PIN * 2));
	UARTSLK_GPIO_PORT->MODER |= (0x2 << (UARTSLK_GPIO_AF_TX_PIN * 2));  // 设置为复用功能
	UARTSLK_GPIO_PORT->OSPEEDR &= ~(0x3 << (UARTSLK_GPIO_AF_TX_PIN * 2));
	UARTSLK_GPIO_PORT->OSPEEDR |= (0x2 << (UARTSLK_GPIO_AF_TX_PIN * 2)); // 25MHz速度
	UARTSLK_GPIO_PORT->OTYPER &= ~(0x1 << UARTSLK_GPIO_AF_TX_PIN);      // 推挽输出

	// 设置复用功能
	if (UARTSLK_GPIO_AF_TX_PIN < 8) {
		UARTSLK_GPIO_PORT->AFR[0] &= ~(0xF << (UARTSLK_GPIO_AF_TX_PIN * 4));
		UARTSLK_GPIO_PORT->AFR[0] |= (UARTSLK_GPIO_AF_TX << (UARTSLK_GPIO_AF_TX_PIN * 4));
	} else {
		UARTSLK_GPIO_PORT->AFR[1] &= ~(0xF << ((UARTSLK_GPIO_AF_TX_PIN - 8) * 4));
		UARTSLK_GPIO_PORT->AFR[1] |= (UARTSLK_GPIO_AF_TX << ((UARTSLK_GPIO_AF_TX_PIN - 8) * 4));
	}

	if (UARTSLK_GPIO_AF_RX_PIN < 8) {
		UARTSLK_GPIO_PORT->AFR[0] &= ~(0xF << (UARTSLK_GPIO_AF_RX_PIN * 4));
		UARTSLK_GPIO_PORT->AFR[0] |= (UARTSLK_GPIO_AF_RX << (UARTSLK_GPIO_AF_RX_PIN * 4));
	} else {
		UARTSLK_GPIO_PORT->AFR[1] &= ~(0xF << ((UARTSLK_GPIO_AF_RX_PIN - 8) * 4));
		UARTSLK_GPIO_PORT->AFR[1] |= (UARTSLK_GPIO_AF_RX << ((UARTSLK_GPIO_AF_RX_PIN - 8) * 4));
	}

	// 配置USART
	UARTSLK_TYPE->CR1 = 0;  // 复位CR1
	UARTSLK_TYPE->CR2 = 0;  // 复位CR2
	UARTSLK_TYPE->CR3 = 0;  // 复位CR3

	// 设置波特率，获取实际的APB1时钟频率
	// 这里APB1时钟频率为48MHz，波特率为1Mbps
	// uint32_t pclk1 = 48000000;
	// uint32_t baud_rate = 1000000;
	// uint32_t usart_div = 0;
	
	// // 计算USART分频值
	// usart_div = (48000000 + (1000000/2)) / 1000000;
	UARTSLK_TYPE->BRR = (48000000 + (1000000/2)) / 1000000;

	// 使能发送和接收，使能RXNE中断
	UARTSLK_TYPE->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);

	// 配置NVIC
	NVIC_SetPriority(UARTSLK_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(UARTSLK_IRQ);

	// 配置TXEN引脚(NRF流控制)
	RCC->AHB1ENR |= UARTSLK_TXEN_PERIF;  // 使能GPIO时钟

	// 配置为输入，上拉
	UARTSLK_TXEN_PORT->MODER &= ~(0x3 << (UARTSLK_TXEN_PIN * 2));      // 输入模式
	UARTSLK_TXEN_PORT->PUPDR &= ~(0x3 << (UARTSLK_TXEN_PIN * 2));
	UARTSLK_TXEN_PORT->PUPDR |= (0x1 << (UARTSLK_TXEN_PIN * 2));       // 上拉

	// 配置EXTI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // 重要：需要先使能SYSCFG时钟，否则无法配置EXTI
	SYSCFG->EXTICR[UARTSLK_TXEN_PIN / 4] &= ~(0xF << ((UARTSLK_TXEN_PIN % 4) * 4));
	SYSCFG->EXTICR[UARTSLK_TXEN_PIN / 4] |= (UARTSLK_TXEN_PORT_SOURCE << ((UARTSLK_TXEN_PIN % 4) * 4));

	// 配置EXTI线
	EXTI->IMR |= (1 << UARTSLK_TXEN_PIN);  // 使能中断
	EXTI->RTSR |= (1 << UARTSLK_TXEN_PIN); // 上升沿触发
	EXTI->FTSR |= (1 << UARTSLK_TXEN_PIN); // 下降沿触发
	EXTI->PR = (1 << UARTSLK_TXEN_PIN);    // 清除挂起标志

	NVIC_EnableIRQ(EXTI0_IRQn);

	// 使能USART
	UARTSLK_TYPE->CR1 |= USART_CR1_UE;
	
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
		// 修正位检测逻辑
		while((UARTSLK_TXEN_PORT->IDR & (1 << UARTSLK_TXEN_PIN)) != 0);
	#endif
		while (!(UARTSLK_TYPE->SR & USART_SR_TXE));
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
	UARTSLK_TYPE->CR1 |= USART_CR1_TXEIE;  // 使能TXE中断
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

	// 读取SR后读取DR，避免overrun错误
	uint32_t sr = UARTSLK_TYPE->SR;
	
	if ((sr & USART_SR_RXNE) != 0) /*接收非空中断*/
	{
		u8 rxDataInterrupt = (u8)(UARTSLK_TYPE->DR & 0xFF);
		xQueueSendFromISR(uartslkDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
	}
	else if ((sr & USART_SR_TXE) && (UARTSLK_TYPE->CR1 & USART_CR1_TXEIE))
	{
		if (outDataIsr && (dataIndexIsr < dataSizeIsr))
		{
			UARTSLK_TYPE->DR = outDataIsr[dataIndexIsr] & 0x00FF;
			dataIndexIsr++;
		} else
		{
			UARTSLK_TYPE->CR1 &= ~USART_CR1_TXEIE;  // 禁用TXE中断
			xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
		}
	}
	
	// 确保任务切换
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void __attribute__((used)) USART2_IRQHandler(void)	
{
	uartslkIsr();
}

