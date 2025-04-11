#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "atkp.h"
#include "radiolink.h"
#include "stabilizer.h"
#include "motors.h"
#include "commander.h"
#include "pm.h"
#include "pid.h"
#include "attitude_pid.h"
#include "position_pid.h"
#include "config_param.h"
#include "power_control.h"
#include "remoter_ctrl.h"
#include "state_estimator.h"
#include "communicate.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

//数据拆分宏定义
#define  BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )


#define ATKP_RX_QUEUE_SIZE 	10 /*ATKP包接收队列消息个数*/

typedef struct  
{
	u16 roll;
	u16 pitch;
	u16 yaw;
	u16 thrust;
}joystickFlyui16_t;


bool isConnect = false;
bool isInit = false;
bool flyable = true;
static joystickFlyui16_t rcdata;
static xQueueHandle rxQueue;

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;
extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;


static void atkpReceiveAnl(atkp_t *anlPacket)
{
	if(anlPacket->msgID == DOWN_REMOTER)/*遥控器*/	
	{
		atkp_write(anlPacket);	/*发送数据到stablilizer通信的队列*/
	}
} 

void atkpRxAnlTask(void *param)
{
	atkp_t p;
	while(1)
	{
		xQueueReceive(rxQueue, &p, portMAX_DELAY);
		atkpReceiveAnl(&p);
	}
}

void atkpInit(void)
{
	if(isInit) return;
	rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(rxQueue);
	isInit = true;
}

bool atkpReceivePacketBlocking(atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}
