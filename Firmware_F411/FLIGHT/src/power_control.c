#include "power_control.h"
#include "motors.h"
#include "atkp.h"


static bool motorSetEnable = false;
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet={0, 0, 0, 0};


void powerControlInit(void)
{
	motorsInit();
}

void powerDataprocess(atkp_t* anlPacket)
{
	s16 enable = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
	s16 m1_set = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
	s16 m2_set = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
	s16 m3_set = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
	s16 m4_set = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
	setMotorPWM(enable,m1_set,m2_set,m3_set,m4_set);
}

bool powerControlTest(void)
{
	bool pass = true;

	pass &= motorsTest();

	return pass;
}

u16 limitThrust(int value)
{
	if(value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if(value < 0)
	{
		value = 0;
	}

	return (u16)value;
}

void powerControl(control_t *control)	/*功率输出控制*/
{
	s16 r = control->roll / 2.0f;
	s16 p = control->pitch / 2.0f;
	
	motorPWM.m1 = limitThrust(control->thrust - r - p + control->yaw);
	motorPWM.m2 = limitThrust(control->thrust - r + p - control->yaw);
	motorPWM.m3 = limitThrust(control->thrust + r + p + control->yaw);
	motorPWM.m4 = limitThrust(control->thrust + r - p - control->yaw);		

	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	motorsSetRatio(MOTOR_M1, motorPWM.m1);	/*控制电机输出百分比*/
	motorsSetRatio(MOTOR_M2, motorPWM.m2);
	motorsSetRatio(MOTOR_M3, motorPWM.m3);
	motorsSetRatio(MOTOR_M4, motorPWM.m4);
}

void getMotorPWM(motorPWM_t* get)
{
	*get = motorPWM;
}

void setMotorPWM(bool enable, u32 m1_set, u32 m2_set, u32 m3_set, u32 m4_set)
{
	motorSetEnable = enable;
	motorPWMSet.m1 = m1_set;
	motorPWMSet.m2 = m2_set;
	motorPWMSet.m3 = m3_set;	
	motorPWMSet.m4 = m4_set;
}
