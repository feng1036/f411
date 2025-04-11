#include <stdbool.h>
#include "pid.h"
#include "sensors.h"
#include "atkp.h"
#include "attitude_pid.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 姿态PID控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 纠正角度环和角速度环积分时间参数错误的bug。
********************************************************************************/

/*角度环积分限幅*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT   30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT     180.0

/*角速度环积分限幅*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		50.0

PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

void attitudeControlInit(float ratePidDt, float anglePidDt)
{
	pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, anglePidDt);			/*roll  角度PID初始化*/
	pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, anglePidDt);			/*pitch 角度PID初始化*/
	pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, anglePidDt);				/*yaw   角度PID初始化*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);		/*roll  角度积分限幅设置*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);		/*pitch 角度积分限幅设置*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);			/*yaw   角度积分限幅设置*/
	
	pidInit(&pidRateRoll, 0, configParam.pidRate.roll, ratePidDt);				/*roll  角速度PID初始化*/
	pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, ratePidDt);			/*pitch 角速度PID初始化*/
	pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, ratePidDt);				/*yaw   角速度PID初始化*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);			/*roll  角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);		/*pitch 角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);			/*yaw   角速度积分限幅设置*/
}

void attitudeDataprocess(atkp_t* anlPacket){
	if(anlPacket->msgID == DOWN_PID1)
	{
		pidRateRoll.kp  = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidRateRoll.ki  = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidRateRoll.kd  = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidRatePitch.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidRatePitch.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidRatePitch.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidRateYaw.kp   = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidRateYaw.ki   = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidRateYaw.kd   = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
	}
	else if(anlPacket->msgID == DOWN_PID2)
	{
		pidAngleRoll.kp  = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidAngleRoll.ki  = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidAngleRoll.kd  = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidAnglePitch.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidAnglePitch.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidAnglePitch.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidAngleYaw.kp   = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidAngleYaw.ki   = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidAngleYaw.kd   = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
//		u8 cksum = atkpCheckSum(anlPacket);
//		sendCheck(anlPacket->msgID,cksum);
	}
}

bool attitudeControlTest()
{
	return true;
}

void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output)	/* 角速度环PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->x));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
}

void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate)	/* 角度环PID */
{
	outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

	float yawError = desiredAngle->yaw - actualAngle->yaw ;
	if (yawError > 180.0f) 
		yawError -= 360.0f;
	else if (yawError < -180.0) 
		yawError += 360.0f;
	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidAngleRoll);
}

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidAnglePitch);
}

void attitudeResetAllPID(void)	/*复位PID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}

void attitudePIDwriteToConfigParam(void)
{
	configParam.pidAngle.roll.kp = pidAngleRoll.kp;
	configParam.pidAngle.roll.ki = pidAngleRoll.ki;
	configParam.pidAngle.roll.kd = pidAngleRoll.kd;
	
	configParam.pidAngle.pitch.kp = pidAnglePitch.kp;
	configParam.pidAngle.pitch.ki = pidAnglePitch.ki;
	configParam.pidAngle.pitch.kd = pidAnglePitch.kd;
	
	configParam.pidAngle.yaw.kp = pidAngleYaw.kp;
	configParam.pidAngle.yaw.ki = pidAngleYaw.ki;
	configParam.pidAngle.yaw.kd = pidAngleYaw.kd;
	
	configParam.pidRate.roll.kp = pidRateRoll.kp;
	configParam.pidRate.roll.ki = pidRateRoll.ki;
	configParam.pidRate.roll.kd = pidRateRoll.kd;
	
	configParam.pidRate.pitch.kp = pidRatePitch.kp;
	configParam.pidRate.pitch.ki = pidRatePitch.ki;
	configParam.pidRate.pitch.kd = pidRatePitch.kd;
	
	configParam.pidRate.yaw.kp = pidRateYaw.kp;
	configParam.pidRate.yaw.ki = pidRateYaw.ki;
	configParam.pidRate.yaw.kd = pidRateYaw.kd;
}

