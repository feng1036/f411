#include <stdbool.h>
#include "console.h"
#include "atkp.h"
#include "radiolink.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "semphr.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 数据打印驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


atkp_t messageToPrint;
xSemaphoreHandle synch = NULL;

static const char fullMsg[] = "<F>\n";
static bool isInit;


void consoleInit()
{
	if (isInit) return;

	messageToPrint.msgID = UP_PRINTF;
	messageToPrint.dataLen = 0;
	vSemaphoreCreateBinary(synch);

	isInit = true;
}

bool consoleTest(void)
{
	return isInit;
}
/* 输入一个字符到console缓冲区*/
int consolePutchar(int ch)
{
	int i;
	bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

	if (!isInit) 
	{
		return 0;
	}
	if (isInInterrupt) 
	{
		return consolePutcharFromISR(ch);
	}
	if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
	{
		if (messageToPrint.dataLen < ATKP_MAX_DATA_SIZE)	/*#define ATKP_MAX_DATA_SIZE 30*/
		{
			messageToPrint.data[messageToPrint.dataLen] = (u8)ch;
			messageToPrint.dataLen++;
		}
		if (ch == '\n' || messageToPrint.dataLen >= ATKP_MAX_DATA_SIZE)	/*#define ATKP_MAX_DATA_SIZE 30*/
		{
			if (radiolinkGetFreeTxQueuePackets() == 1)
			{
				for (i = 0; i < sizeof(fullMsg) && (messageToPrint.dataLen - i) > 0; i++)
				{
					messageToPrint.data[messageToPrint.dataLen - i] = (u8)fullMsg[sizeof(fullMsg) - i - 1];
				}
			}
			radiolinkSendPacket(&messageToPrint);
			messageToPrint.dataLen = 0;
		}
		xSemaphoreGive(synch);
	}
	return (u8)ch;
}
/* 中断方式输入一个字符到console缓冲区*/
int consolePutcharFromISR(int ch) 
{
	BaseType_t higherPriorityTaskWoken;

	if (xSemaphoreTakeFromISR(synch, &higherPriorityTaskWoken) == pdTRUE) 
	{
		if (messageToPrint.dataLen < ATKP_MAX_DATA_SIZE)
		{
			messageToPrint.data[messageToPrint.dataLen] = (u8)ch;
			messageToPrint.dataLen++;
		}
		xSemaphoreGiveFromISR(synch, &higherPriorityTaskWoken);
	}

	return ch;
}
/* 输入一个字符串到console缓冲区*/
int consolePuts(char *str)
{
	int ret = 0;

	while(*str)
		ret |= consolePutchar(*str++);

	return ret;
}

