#include <math.h>
#include "state_control.h"
#include "stabilizer.h"
#include "position_pid.h"
#include "config_param.h"
#include <stdbool.h>
#include "pid.h"
#include "sensors.h"
#include "atkp.h"

static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

//attitude_pid.c 中的内容

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


void stateControlInit(void)
{
	attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT); /*初始化姿态PID*/	
	positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*初始化位置PID*/
}

void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const u32 tick)
{
	static u16 cnt = 0;
	
	if (RATE_DO_EXECUTE(POSITION_PID_RATE, tick))
	{
		if (setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable)
		{
			positionController(&actualThrust, &attitudeDesired, setpoint, state, POSITION_PID_DT);
		}
	}
	
	//角度环（外环）
	if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick))
	{
		if (setpoint->mode.z == modeDisable)
		{
			actualThrust = setpoint->thrust;
		}
		if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) 
		{
			attitudeDesired.roll = setpoint->attitude.roll;
			attitudeDesired.pitch = setpoint->attitude.pitch;
		}
		
		if(control->flipDir == CENTER)
		{
			attitudeDesired.yaw += setpoint->attitude.yaw/ANGEL_PID_RATE; /*期望YAW 速率模式*/
			if(attitudeDesired.yaw > 180.0f) 
				attitudeDesired.yaw -= 360.0f;
			if(attitudeDesired.yaw < -180.0f) 
				attitudeDesired.yaw += 360.0f;
		}
			
		attitudeDesired.roll += configParam.trimR;	//叠加微调值
		attitudeDesired.pitch += configParam.trimP;		
		
		attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);
	}
	
	//角速度环（内环）
	if (RATE_DO_EXECUTE(RATE_PID_RATE, tick))
	{
		if (setpoint->mode.roll == modeVelocity)
		{
			rateDesired.roll = setpoint->attitudeRate.roll;
			attitudeControllerResetRollAttitudePID();
		}
		if (setpoint->mode.pitch == modeVelocity)
		{
			rateDesired.pitch = setpoint->attitudeRate.pitch;
			attitudeControllerResetPitchAttitudePID();
		}
		
		attitudeRatePID(&sensors->gyro, &rateDesired, control);
	}

	control->thrust = actualThrust;	
	
	if (control->thrust < 5.f)
	{			
		control->roll = 0;
		control->pitch = 0;
		control->yaw = 0;
		
		attitudeResetAllPID();	/*复位姿态PID*/	
		positionResetAllPID();	/*复位位置PID*/
		attitudeDesired.yaw = state->attitude.yaw;		/*复位计算的期望yaw值*/
		
		if(cnt++ > 1500)
		{
			cnt = 0;
		}
	}else
	{
		cnt = 0;
	}
}

// attitude_pid.c 中的内容

static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}//state_control

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
}//state_control

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
	}
}//stabilizer

void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output)	/* 角速度环PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->x));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
}//state_control

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
}//state_control

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidAngleRoll);
}//state_control

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidAnglePitch);
}//state_control

void attitudeResetAllPID(void)	/*复位PID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}//state_control

// void attitudePIDwriteToConfigParam(void)
// {
// 	configParam.pidAngle.roll.kp = pidAngleRoll.kp;
// 	configParam.pidAngle.roll.ki = pidAngleRoll.ki;
// 	configParam.pidAngle.roll.kd = pidAngleRoll.kd;
	
// 	configParam.pidAngle.pitch.kp = pidAnglePitch.kp;
// 	configParam.pidAngle.pitch.ki = pidAnglePitch.ki;
// 	configParam.pidAngle.pitch.kd = pidAnglePitch.kd;
	
// 	configParam.pidAngle.yaw.kp = pidAngleYaw.kp;
// 	configParam.pidAngle.yaw.ki = pidAngleYaw.ki;
// 	configParam.pidAngle.yaw.kd = pidAngleYaw.kd;
	
// 	configParam.pidRate.roll.kp = pidRateRoll.kp;
// 	configParam.pidRate.roll.ki = pidRateRoll.ki;
// 	configParam.pidRate.roll.kd = pidRateRoll.kd;
	
// 	configParam.pidRate.pitch.kp = pidRatePitch.kp;
// 	configParam.pidRate.pitch.ki = pidRatePitch.ki;
// 	configParam.pidRate.pitch.kd = pidRatePitch.kd;
	
// 	configParam.pidRate.yaw.kp = pidRateYaw.kp;
// 	configParam.pidRate.yaw.ki = pidRateYaw.ki;
// 	configParam.pidRate.yaw.kd = pidRateYaw.kd;
// }//在attitude_pid.c中，没有用到

