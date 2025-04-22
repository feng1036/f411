#include "sys.h"
#include "usart.h"	
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"      //FreeRTOS ʹ��  
#endif
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
// extern int consolePutchar(int ch);
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
//	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
//	USART1->DR = (u8) ch;   
//	consolePutchar(ch);	/*��ӡ����λ��*/
	return ch;
}
#endif
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound)
{
    u32 temp;
    
    // ʹ��GPIOAʱ�Ӻ�USART1ʱ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // ʹ��GPIOAʱ��
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   // ʹ��USART1ʱ��
    
    // ����GPIOģʽ
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);  // ���ԭģʽ
    GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); // ����Ϊ���ù���
    
    // ����GPIO�ٶ�Ϊ50MHz
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR10_1);
    
    // ����GPIO�������Ϊ����
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10);
    
    // ����GPIO����
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0);
    
    // ����PA9��PA10�ĸ��ù������ӵ�USART1
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4) | 0xF << ((10 - 8) * 4));
    GPIOA->AFR[1] |= (7 << ((9 - 8) * 4) | 7 << ((10 - 8) * 4)); // AF7: USART1
    
    // ���ò�����
    temp = (u32)(SystemCoreClock / 2) / bound;
    USART1->BRR = temp;
    
    // ����USART1������8λ���ݣ�1λֹͣλ����У�飬�շ�ģʽ
    USART1->CR1 = 0;  // ������
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;  // ʹ�ܷ��ͺͽ���
    
    USART1->CR2 = 0;  // 1λֹͣλ
    USART1->CR3 = 0;  // ��Ӳ��������
    
#if EN_USART1_RX
    // ʹ�ܽ����ж�
    USART1->CR1 |= USART_CR1_RXNEIE;
    
    // ����NVIC
    NVIC->ISER[USART1_IRQn >> 5] = 1 << (USART1_IRQn & 0x1F); // ʹ��USART1�ж�
    NVIC->IP[USART1_IRQn] = (15 << 4); // �������ȼ�Ϊ15
#endif
    
    // ʹ��USART1
    USART1->CR1 |= USART_CR1_UE;
}
#endif
