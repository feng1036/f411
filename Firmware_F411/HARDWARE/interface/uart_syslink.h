#ifndef __UART_SYSLINK_H
#define __UART_SYSLINK_H
#include <stdbool.h>
#include "sys.h"


#define UARTSLK_TYPE             USART2	
#define UARTSLK_PERIF            ((uint32_t)0x00020000)
#define ENABLE_UARTSLK_RCC       RCC_APB1PeriphClockCmd
#define UARTSLK_IRQ              USART2_IRQn

#define UARTSLK_GPIO_PERIF       ((uint32_t)0x00000001) 
#define UARTSLK_GPIO_PORT        GPIOA
#define UARTSLK_GPIO_TX_PIN      ((uint16_t)0x0004)
#define UARTSLK_GPIO_RX_PIN      ((uint16_t)0x0008)
#define UARTSLK_GPIO_AF_TX_PIN   ((uint8_t)0x02)
#define UARTSLK_GPIO_AF_RX_PIN   ((uint8_t)0x03)
#define UARTSLK_GPIO_AF_TX       ((uint8_t)0x07)
#define UARTSLK_GPIO_AF_RX       ((uint8_t)0x07)

#define UARTSLK_TXEN_PERIF       ((uint32_t)0x00000001)
#define UARTSLK_TXEN_PORT        GPIOA
#define UARTSLK_TXEN_PIN         ((uint16_t)0x0001)
#define UARTSLK_TXEN_EXTI        ((uint32_t)0x00001)


void uartslkInit(void);		/*���ڳ�ʼ��*/
bool uartslkTest(void);
bool uartslkGetDataWithTimout(u8 *c);	/*�ӽ��ն��ж�ȡ����(����ʱ����)*/
void uartslkSendData(u32 size, u8* data);	/*����ԭʼ����*/
void uartslkSendDataIsrBlocking(u32 size, u8* data);/*�жϷ�ʽ����ԭʼ����*/
int uartslkPutchar(int ch);		/*����һ���ַ�������*/
void uartslkIsr(void);		/*�����жϷ�����*/

#endif /* __UART_SYSLINK_H */
