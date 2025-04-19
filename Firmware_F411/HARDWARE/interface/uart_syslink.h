#ifndef __UART_SYSLINK_H
#define __UART_SYSLINK_H
#include <stdbool.h>
#include "sys.h"


#define UARTSLK_TYPE             USART2	
#define UARTSLK_PERIF            RCC_APB1Periph_USART2
#define ENABLE_UARTSLK_RCC       RCC_APB1PeriphClockCmd
#define UARTSLK_IRQ              USART2_IRQn

#define UARTSLK_GPIO_PERIF       RCC_AHB1Periph_GPIOA 
#define UARTSLK_GPIO_PORT        GPIOA
#define UARTSLK_GPIO_TX_PIN      GPIO_Pin_2
#define UARTSLK_GPIO_RX_PIN      GPIO_Pin_3
#define UARTSLK_GPIO_AF_TX_PIN   GPIO_PinSource2
#define UARTSLK_GPIO_AF_RX_PIN   GPIO_PinSource3
#define UARTSLK_GPIO_AF_TX       GPIO_AF_USART2
#define UARTSLK_GPIO_AF_RX       GPIO_AF_USART2

#define UARTSLK_TXEN_PERIF       RCC_AHB1Periph_GPIOA
#define UARTSLK_TXEN_PORT        GPIOA
#define UARTSLK_TXEN_PIN         GPIO_Pin_0
#define UARTSLK_TXEN_EXTI        EXTI_Line0


void uartslkInit(void);		/*串口初始化*/
bool uartslkTest(void);
bool uartslkGetDataWithTimout(u8 *c);	/*从接收队列读取数据(带超时处理)*/
void uartslkSendData(u32 size, u8* data);	/*发送原始数据*/
void uartslkSendDataIsrBlocking(u32 size, u8* data);/*中断方式发送原始数据*/
int uartslkPutchar(int ch);		/*发送一个字符到串口*/
void uartslkIsr(void);		/*串口中断服务函数*/

#endif /* __UART_SYSLINK_H */
