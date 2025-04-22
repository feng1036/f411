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

// USART2��غ궨��(ʹ�ñ�׼�Ĵ��������Ϊ��ȫ)
#define UARTSLK_SR_RXNE        USART_SR_RXNE
#define UARTSLK_SR_TXE         USART_SR_TXE 

// EXTI�����õĶ˿�Դ����
// ������Ҫ����UARTSLK_TXEN_PORTʵ��ʹ�õĶ˿�������
// GPIOA=0, GPIOB=1, GPIOC=2, ��������
#define UARTSLK_TXEN_PORT_SOURCE    0  // ����TXEN������GPIOA�ϣ�����ʵ���������

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

	// ʹ��GPIO��USARTʱ��
	RCC->AHB1ENR |= UARTSLK_GPIO_PERIF;  // ʹ��GPIOʱ��
	RCC->APB1ENR |= UARTSLK_PERIF;       // ʹ��USARTʱ��(������APB1�ϵ�����)

	// ����GPIO�ܽ�
	// ������λ
	UARTSLK_GPIO_PORT->MODER &= ~(0x3 << (UARTSLK_GPIO_AF_RX_PIN * 2));
	UARTSLK_GPIO_PORT->MODER |= (0x2 << (UARTSLK_GPIO_AF_RX_PIN * 2));  // ����Ϊ���ù���
	UARTSLK_GPIO_PORT->PUPDR &= ~(0x3 << (UARTSLK_GPIO_AF_RX_PIN * 2));
	UARTSLK_GPIO_PORT->PUPDR |= (0x1 << (UARTSLK_GPIO_AF_RX_PIN * 2));  // ����

	// ����TX����
	UARTSLK_GPIO_PORT->MODER &= ~(0x3 << (UARTSLK_GPIO_AF_TX_PIN * 2));
	UARTSLK_GPIO_PORT->MODER |= (0x2 << (UARTSLK_GPIO_AF_TX_PIN * 2));  // ����Ϊ���ù���
	UARTSLK_GPIO_PORT->OSPEEDR &= ~(0x3 << (UARTSLK_GPIO_AF_TX_PIN * 2));
	UARTSLK_GPIO_PORT->OSPEEDR |= (0x2 << (UARTSLK_GPIO_AF_TX_PIN * 2)); // 25MHz�ٶ�
	UARTSLK_GPIO_PORT->OTYPER &= ~(0x1 << UARTSLK_GPIO_AF_TX_PIN);      // �������

	// ���ø��ù���
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

	// ����USART
	UARTSLK_TYPE->CR1 = 0;  // ��λCR1
	UARTSLK_TYPE->CR2 = 0;  // ��λCR2
	UARTSLK_TYPE->CR3 = 0;  // ��λCR3

	// ���ò����ʣ���ȡʵ�ʵ�APB1ʱ��Ƶ��
	// ����APB1ʱ��Ƶ��Ϊ48MHz��������Ϊ1Mbps
	// uint32_t pclk1 = 48000000;
	// uint32_t baud_rate = 1000000;
	// uint32_t usart_div = 0;
	
	// // ����USART��Ƶֵ
	// usart_div = (48000000 + (1000000/2)) / 1000000;
	UARTSLK_TYPE->BRR = (48000000 + (1000000/2)) / 1000000;

	// ʹ�ܷ��ͺͽ��գ�ʹ��RXNE�ж�
	UARTSLK_TYPE->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);

	// ����NVIC
	NVIC_SetPriority(UARTSLK_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(UARTSLK_IRQ);

	// ����TXEN����(NRF������)
	RCC->AHB1ENR |= UARTSLK_TXEN_PERIF;  // ʹ��GPIOʱ��

	// ����Ϊ���룬����
	UARTSLK_TXEN_PORT->MODER &= ~(0x3 << (UARTSLK_TXEN_PIN * 2));      // ����ģʽ
	UARTSLK_TXEN_PORT->PUPDR &= ~(0x3 << (UARTSLK_TXEN_PIN * 2));
	UARTSLK_TXEN_PORT->PUPDR |= (0x1 << (UARTSLK_TXEN_PIN * 2));       // ����

	// ����EXTI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // ��Ҫ����Ҫ��ʹ��SYSCFGʱ�ӣ������޷�����EXTI
	SYSCFG->EXTICR[UARTSLK_TXEN_PIN / 4] &= ~(0xF << ((UARTSLK_TXEN_PIN % 4) * 4));
	SYSCFG->EXTICR[UARTSLK_TXEN_PIN / 4] |= (UARTSLK_TXEN_PORT_SOURCE << ((UARTSLK_TXEN_PIN % 4) * 4));

	// ����EXTI��
	EXTI->IMR |= (1 << UARTSLK_TXEN_PIN);  // ʹ���ж�
	EXTI->RTSR |= (1 << UARTSLK_TXEN_PIN); // �����ش���
	EXTI->FTSR |= (1 << UARTSLK_TXEN_PIN); // �½��ش���
	EXTI->PR = (1 << UARTSLK_TXEN_PIN);    // ��������־

	NVIC_EnableIRQ(EXTI0_IRQn);

	// ʹ��USART
	UARTSLK_TYPE->CR1 |= USART_CR1_UE;
	
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
		// ����λ����߼�
		while((UARTSLK_TXEN_PORT->IDR & (1 << UARTSLK_TXEN_PIN)) != 0);
	#endif
		while (!(UARTSLK_TYPE->SR & USART_SR_TXE));
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
	UARTSLK_TYPE->CR1 |= USART_CR1_TXEIE;  // ʹ��TXE�ж�
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

	// ��ȡSR���ȡDR������overrun����
	uint32_t sr = UARTSLK_TYPE->SR;
	
	if ((sr & USART_SR_RXNE) != 0) /*���շǿ��ж�*/
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
			UARTSLK_TYPE->CR1 &= ~USART_CR1_TXEIE;  // ����TXE�ж�
			xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
		}
	}
	
	// ȷ�������л�
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void __attribute__((used)) USART2_IRQHandler(void)	
{
	uartslkIsr();
}

