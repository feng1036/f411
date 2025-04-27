#include "sys.h"
#include "usart.h"	
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"      //FreeRTOS 使用  
#endif
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
// extern int consolePutchar(int ch);
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART1->DR = (u8) ch;   
//	consolePutchar(ch);	/*打印到上位机*/
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound)
{
    u32 temp;
    
    // 使能GPIOA时钟和USART1时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // 使能GPIOA时钟
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   // 使能USART1时钟
    
    // 设置GPIO模式
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);  // 清除原模式
    GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); // 设置为复用功能
    
    // 设置GPIO速度为50MHz
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR10_1);
    
    // 设置GPIO输出类型为推挽
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10);
    
    // 设置GPIO上拉
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0);
    
    // 设置PA9和PA10的复用功能连接到USART1
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4) | 0xF << ((10 - 8) * 4));
    GPIOA->AFR[1] |= (7 << ((9 - 8) * 4) | 7 << ((10 - 8) * 4)); // AF7: USART1
    
    // 设置波特率
    temp = (u32)(SystemCoreClock / 2) / bound;
    USART1->BRR = temp;
    
    // 配置USART1参数：8位数据，1位停止位，无校验，收发模式
    USART1->CR1 = 0;  // 先清零
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;  // 使能发送和接收
    
    USART1->CR2 = 0;  // 1位停止位
    USART1->CR3 = 0;  // 无硬件流控制
    
#if EN_USART1_RX
    // 使能接收中断
    USART1->CR1 |= USART_CR1_RXNEIE;
    
    // 配置NVIC
    NVIC->ISER[USART1_IRQn >> 5] = 1 << (USART1_IRQn & 0x1F); // 使能USART1中断
    NVIC->IP[USART1_IRQn] = (15 << 4); // 设置优先级为15
#endif
    
    // 使能USART1
    USART1->CR1 |= USART_CR1_UE;
}
#endif
