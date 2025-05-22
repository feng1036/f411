/******************************************************************************* 
 * 文件名:   communicate.c
 * 作者:     赵薪宇
 * 创建日期: 2025/4/1
 * 
 * 描述:     与稳定器(stabilizer)进行通信的模块
 *           定义了通信协议、数据收发队列和相关处理函数
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "communicate.h"
#include "rvm.h"
#include "rmp.h"
#include "prc_remote.h"
/* End Include ***************************************************************/

#define GPIO_Pin_0                 ((uint16_t)0x0001)
#define GPIO_Pin_2                 ((uint16_t)0x0004)
#define GPIO_Pin_3                 ((uint16_t)0x0008)
#define CR1_CLEAR_MASK            ((uint16_t)0x160C)
#define CR3_CLEAR_MASK            ((uint16_t)0x0300)

#define UARTSLK_DATA_TIMEOUT_MS 	1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((uint_32)0x00000001)

#define RADIO_MSG ((volatile struct remoteData_t*)DATA_SHARED_REMOTE_BASE)

#define POOL_SIZE 128

/* Global ****************************************************************/

/* 通信状态机状态枚举 */
static enum {
    waitForStartByte1,  /* 等待第一个起始字节 */
    waitForStartByte2,  /* 等待第二个起始字节 */
    waitForMsgID,       /* 等待消息ID */
    waitForDataLength,  /* 等待数据长度 */
    waitForData,        /* 等待数据 */
    waitForChksum1      /* 等待校验和 */
} state;

struct Message_Uart{
    struct RMP_List Head;
    uint8_t uartData;
};

struct RMP_List Pool_Uart;
struct Message_Uart msgarray_uart[POOL_SIZE];

//注册消息队列
volatile struct RMP_Msgq Queue_Uart;

/* 通信数据包缓存 */
struct remoteData_t packet;

bool isInit = false;

uint8_t *outDataIsr;
uint8_t dataIndexIsr;
uint8_t dataSizeIsr;

/* End Global ****************************************************************/


/* Function:uart_data_Read **************************************************
Description : Receive data from Uart.
Input       : Pointer to data.
Output      : None.
Return      : None.
******************************************************************************/
bool uart_data_Read(uint8_t* uartdata)
{
	volatile struct Message_Uart* msg_S; 
	if(RMP_Msgq_Rcv(&Queue_Uart,(struct RMP_List**)(&msg_S),1000)!=0)
		return false;
	*uartdata=msg_S->uartData;
	RMP_INT_MASK();
	RMP_List_Ins((struct RMP_List*)(&msg_S),Pool_Uart.Prev,Pool_Uart.Next);
	RMP_INT_UNMASK();
	return true;
}

/* Function:Contact_Uart ****************************************************
Description : Interrupt handling program with Uart.
Input       : None.
Output      : None.
Return      : None.
******************************************************************************/
void Send_From_Uart(void)
{
	volatile struct Message_Uart* msg_s;
	
	if(Pool_Uart.Next==&Pool_Uart) return;
	msg_s=(volatile struct Message_Uart*)(Pool_Uart.Next);
	RMP_List_Del(msg_s->Head.Prev,msg_s->Head.Next);
	msg_s->uartData=(uint8_t)(USART2->DR & 0xFF);
	RMP_Msgq_Snd_ISR(&Queue_Uart,msg_s);
}

/* Function:UART2_IRQHandler ****************************************************
Description : Uart interrupt handling program.
Input       : None.
Output      : None.
Return      : None.
******************************************************************************/
void UART2_IRQHandler(void)
{
    Send_From_Uart();
}

/* Function:uartslkGetDataWithTimout ****************************************************
Description : Receive data with timeout.
Input       : Uint8_t pointer pointing to the data.
Output      : None.
Return      : None.
******************************************************************************/
bool uartslkGetDataWithTimout(uint8_t *c)
{
	if (uart_data_Read(c) == true)
	{
		return true;
	}
	*c = 0;
	return false;
}

void uartslkInit(void)	/*´®¿Ú³õÊ¼»¯*/
{	
	/* Ê¹ÄÜGPIO ºÍ UART Ê±ÖÓ*/
	RCC->AHB1ENR |= (uint32_t)0x00000001;
	RCC->APB1ENR |= (uint32_t)0x00020000;

	/* ÅäÖÃUSART RxÎª¸¡¿ÕÊäÈë*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER3);
	GPIOA->MODER |= (uint32_t)(0x80);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3);
	GPIOA->OTYPER  &= ~((GPIO_OTYPER_OT_3));
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	GPIOA->PUPDR |= (uint32_t)(0x40);

	// /* ÅäÖÃUSART Tx ¸´ÓÃ¹¦ÄÜÊä³ö*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);
	GPIOA->MODER |= (uint32_t)(0x20);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR2);
    GPIOA->OSPEEDR |= (uint32_t)(0x10);
	GPIOA->OTYPER  &= ~((GPIO_OTYPER_OT_2));
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	GPIOA->PUPDR |= (uint32_t)(0x10);

	/*¶Ë¿ÚÓ³Éä*/
	GPIOA->AFR[0]&=~((uint32_t)0xF<<(8U));
	GPIOA->AFR[0]|=((uint32_t)(0x07)<<(8U));
	GPIOA->AFR[0]&=~((uint32_t)0xF<<(12U));
	GPIOA->AFR[0]|=((uint32_t)(0x7)<<(12U));
	
	USART2->CR2 &= ~USART_CR2_STOP;
	USART2->CR1 &= ~CR1_CLEAR_MASK;
	USART2->CR1 |=0x000C;
	USART2->CR3 &=~CR3_CLEAR_MASK;
	USART2->BRR = (uint16_t)48;

	// /*ÅäÖÃ´®¿Ú·Ç¿ÕÖÐ¶Ï*/
    NVIC->IP[USART2_IRQn]=(uint32_t)(((5U)&0x0F)<<4);
    NVIC->ISER[USART2_IRQn>>0x05]=((uint32_t)1U<<(USART2_IRQn&(uint8_t)0x1F));
	/*´®¿Ú½ÓÊÕÊý¾Ý¼Ä´æÆ÷·Ç¿ÕÖÐ¶Ï*/
	USART2->CR1|=(uint32_t)(0x20);
	/*ÅäÖÃTXENÒý½Å(NRFÁ÷¿ØÖÆ)*/
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

bool RemoteTest(void)
{
	return isInit;
}

/**
 * @brief 通信处理任务
 * 
 * 持续处理接收到的通信数据，按照协议解析并存入队列
 * 
 * @param param 任务参数（未使用）
 */
void RemoteTask(void *param)
{
	uartslkInit();

    //初始化内存池
	RMP_List_Crt(&Pool_Uart);

	for(int i=0;i<POOL_SIZE;i++){
		RMP_List_Ins(&msgarray_uart[i].Head,&Pool_Uart.Prev,&Pool_Uart.Next);
	}

	//初始化消息队列
	RMP_Msgq_Crt(&Queue_Uart);

    /* 初始化状态机 */
    state = waitForStartByte1;

    uint8_t c;                  /* 当前接收的字节 */
    uint8_t dataIndex = 0;      /* 数据缓冲区索引 */
    uint8_t checksum = 0;       /* 校验和计算 */

    
    /* 任务主循环 */
    while(1)
    {
        /* 尝试从UART接收一个字节，带超时 */
        if (uartslkGetDataWithTimout(&c))
        {
            /* 根据当前状态处理接收到的字节 */
            switch(state)
            {
                case waitForStartByte1:
                    /* 检查第一个起始字节 */
                    checksum = c;
                    state = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
                    break;
                    
                case waitForStartByte2:
                    /* 检查第二个起始字节 */
                    checksum += c;
                    state = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
                    break;
                    
                case waitForMsgID:
                    /* 获取消息ID */
                    packet.msgID = c;
                    checksum += c;
                    state = waitForDataLength;
                    break;
                    
                case waitForDataLength:
                    /* 获取数据长度字段 */
                    if (c <= RADIO_MAX_DATA_SIZE)
                    {
                        packet.dataLen = c;
                        dataIndex = 0;
                        checksum += c;
                        /* 如果数据长度为0，直接等待校验和 */
                        state = (c > 0) ? waitForData : waitForChksum1;
                    } 
                    else 
                    {
                        /* 数据长度超出范围，重新开始 */
                        state = waitForStartByte1;
                    }
                    break;
                    
                case waitForData:
                    /* 接收数据字段 */
                    packet.data[dataIndex] = c;
                    dataIndex++;
                    checksum += c;
                    
                    /* 数据接收完成后，进入校验和状态 */
                    if (dataIndex == packet.dataLen)
                    {
                        state = waitForChksum1;
                    }
                    break;
                    
                case waitForChksum1:
                    /* 验证校验和 */
                    if (checksum == c)
                    {
                        /* 校验成功，写入共享内存 */
                        *RADIO_MSG = packet;
						RVM_Hyp_Evt_Snd(22);
                    }
                    /* 无论校验是否成功，都重新开始接收新数据 */
                    state = waitForStartByte1;
                    break;
                    
                default:
                    break;
            }
        }
        else
        {
            /* 接收超时，重置状态机 */
            state = waitForStartByte1;
        }
    }
}
