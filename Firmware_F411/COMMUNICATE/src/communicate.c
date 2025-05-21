//和stablizier进行通信的代码
// Created by zhaoxinyu on 2025/4/1.

/********************************************************************************	 
 *文件名：	communicate_with_stablilizer.cpp
 *作者：	赵薪宇
 *描述：	和stablilizer进行通信的代码,定义了如何与stablilizer通信,包括收发信息的队列,收发函数等
********************************************************************************/
#include "communicate.h"
#include "uart_syslink.h"
#include <stdbool.h>

static xQueueHandle atkp_stabilizer_queue;	/*和stablilizer通信的队列*/
static xQueueHandle sensor_data_queue;
static xQueueHandle sensor_gyroBias_queue;

static bool isInit = false;

#define RADIOLINK_TX_QUEUE_SIZE  30 /*接收队列个数*/

static enum
{
	waitForStartByte1,
	waitForStartByte2,
	waitForMsgID,
	waitForDataLength,
	waitForData,
	waitForChksum1,
}rxState;
static atkp_t rxPacket;

void communicateInit(void)
{
    if(isInit) return;
	sensor_data_queue=xQueueCreate(1,12*sizeof(float));
	sensor_gyroBias_queue=xQueueCreate(1,sizeof(bool));
    atkp_stabilizer_queue = xQueueCreate(1, sizeof(atkp_t));	/*创建和stablilizer通信的队列*/
    ASSERT(atkp_stabilizer_queue);
    isInit = true;
}

void atkp_write(atkp_t *p)
{
    xQueueSend(atkp_stabilizer_queue, p, 0);	/*发送遥控数据到stablilizer通信的队列*/
}

BaseType_t atkp_read(atkp_t *p)
{
    return xQueueReceive(atkp_stabilizer_queue, p, 0);	/*接收数据从stablilizer通信的队列*/
}

BaseType_t sensor_data_Read(sensorData_t* sensordata)
{
    return xQueueReceive(sensor_data_queue,sensordata,0);
}

void sensor_data_Write(sensorData_t* sensordata)
{
    xQueueOverwrite(sensor_data_queue,sensordata);
}

BaseType_t sensor_gyroBias_Read(bool* gyroBias)
{
    return xQueueReceive(sensor_gyroBias_queue,gyroBias,0);
}

void sensor_gyroBias_Write(bool* gyroBias)
{
    xQueueOverwrite(sensor_gyroBias_queue,gyroBias);
}

//radiolink接收ATKPPacket任务
void radiolinkTask(void *param)
{
	rxState = waitForStartByte1;
	
	u8 c;
	u8 dataIndex = 0;
	u8 cksum = 0;

	while(1)
	{
		if (uartslkGetDataWithTimout(&c))
		{
			switch(rxState)
			{
				case waitForStartByte1:
					rxState = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
					cksum = c;
					break;
				case waitForStartByte2:
					rxState = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
					cksum += c;
					break;
				case waitForMsgID:
					rxPacket.msgID = c;
					rxState = waitForDataLength;
					cksum += c;
					break;
				case waitForDataLength:
					if (c <= ATKP_MAX_DATA_SIZE)
					{
						rxPacket.dataLen = c;
						dataIndex = 0;
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,数据长度为0，校验1*/
						cksum += c;
					} else 
					{
						rxState = waitForStartByte1;
					}
					break;
				case waitForData:
					rxPacket.data[dataIndex] = c;
					dataIndex++;
					cksum += c;
					if (dataIndex == rxPacket.dataLen)
					{
						rxState = waitForChksum1;
					}
					break;
				case waitForChksum1:
					if (cksum == c)	/*所有校验正确*/
					{
						atkp_write(&rxPacket);
					} 
					else	/*校验错误*/
					{
						rxState = waitForStartByte1;
						IF_DEBUG_ASSERT(1);
					}
					rxState = waitForStartByte1;
					break;
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*超时处理*/
		{
			rxState = waitForStartByte1;
		}
	}
}


