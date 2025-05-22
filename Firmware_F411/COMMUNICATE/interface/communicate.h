#ifndef COMMUNICATE_H
#define COMMUNICATE_H
#include <stdbool.h>
#include <stdint.h>

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

/*下行帧头*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define RADIO_MAX_DATA_SIZE 30

/*通讯数据结构*/
struct remoteData_t
{
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[RADIO_MAX_DATA_SIZE];
};

/*下行指令ID*/
typedef enum 
{
	DOWN_COMMAND	= 0x01,
	DOWN_ACK		= 0x02,
	DOWN_RCDATA		= 0x03,
	DOWN_POWER		= 0x05,
	DOWN_FLYMODE	= 0x0A,
	DOWN_PID1		= 0x10,
	DOWN_PID2		= 0x11,
	DOWN_PID3		= 0x12,
	DOWN_PID4		= 0x13,
	DOWN_PID5		= 0x14,
	DOWN_PID6		= 0x15,
	DOWN_RADIO		= 0x40,
	DOWN_REMOTER	= 0x50,
}downmsgID_e;

bool uart_data_Read(uint8_t* uartdata);

void Send_From_Uart(void);

void UART2_IRQHandler(void);

bool uartslkGetDataWithTimout(uint8_t *c);

void uartslkInit(void);

bool RemoteTest(void);

void RemoteTask(void *param);

#endif
