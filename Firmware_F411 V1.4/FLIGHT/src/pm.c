#include "sys.h"
#include <string.h>
#include <stdbool.h>
#include "config.h"
#include "system.h"
#include "pm.h"
#include "led.h"
#include "ledseq.h"
#include "commander.h"
#include "radiolink.h"
#include "remoter_ctrl.h"
#include "stabilizer.h"
#include "sensfusion6.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 电源管理驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

static float batteryVoltage;

static bool isInit;

void pmInit(void)
{
	if(isInit) return;

	batteryVoltage = 3.7f;
	
	isInit = true;
}

bool pmTest(void)
{
	return isInit;
}

float pmGetBatteryVoltage(void)
{
	return batteryVoltage;
}
