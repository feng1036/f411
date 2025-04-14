#include <math.h>
#include "pid.h"
#include "commander.h"
#include "config_param.h"
#include "position_pid.h"
#include "remoter_ctrl.h"
#include "maths.h"


#define THRUST_BASE  		(20000)	/*��������ֵ*/

#define PIDVX_OUTPUT_LIMIT	120.0f	//ROLL�޷�	(��λ���0.15��ϵ��)
#define PIDVY_OUTPUT_LIMIT	120.0f 	//PITCH�޷�	(��λ���0.15��ϵ��)
#define PIDVZ_OUTPUT_LIMIT	(40000)	/*PID VZ�޷�*/

#define PIDX_OUTPUT_LIMIT	1200.0f	//X���ٶ��޷�(��λcm/s ��0.1��ϵ��)
#define PIDY_OUTPUT_LIMIT	1200.0f	//Y���ٶ��޷�(��λcm/s ��0.1��ϵ��)
#define PIDZ_OUTPUT_LIMIT	120.0f	//Z���ٶ��޷�(��λcm/s)


static float thrustLpf = THRUST_BASE;	/*���ŵ�ͨ*/

PidObject pidVX;
PidObject pidVY;
PidObject pidVZ;

PidObject pidX;
PidObject pidY;
PidObject pidZ;

void positionControlInit(float velocityPidDt, float posPidDt)
{
	pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt);	/*vx PID��ʼ��*/
	pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt);	/*vy PID��ʼ��*/
	pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt);	/*vz PID��ʼ��*/
	pidSetOutputLimit(&pidVX, PIDVX_OUTPUT_LIMIT);		/* ����޷� */
	pidSetOutputLimit(&pidVY, PIDVY_OUTPUT_LIMIT);		/* ����޷� */
	pidSetOutputLimit(&pidVZ, PIDVZ_OUTPUT_LIMIT);		/* ����޷� */
	
	pidInit(&pidX, 0, configParam.pidPos.x, posPidDt);			/*x PID��ʼ��*/
	pidInit(&pidY, 0, configParam.pidPos.y, posPidDt);			/*y PID��ʼ��*/
	pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt);			/*z PID��ʼ��*/
	pidSetOutputLimit(&pidX, PIDX_OUTPUT_LIMIT);		/* ����޷� */
	pidSetOutputLimit(&pidY, PIDY_OUTPUT_LIMIT);		/* ����޷� */
	pidSetOutputLimit(&pidZ, PIDZ_OUTPUT_LIMIT);		/* ����޷� */
}

void positionDataprocess(atkp_t* anlPacket){
	if(anlPacket->msgID == DOWN_PID3)
	{
		pidVZ.kp = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidVZ.ki = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidVZ.kd = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		pidZ.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidZ.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidZ.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		pidVX.kp = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidVX.ki = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidVX.kd = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		
		pidVY = pidVX;	//λ������PID��X\Y������һ����
	}
	else if(anlPacket->msgID == DOWN_PID4)
	{
		pidX.kp = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidX.ki = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidX.kd = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		pidY = pidX;	//λ�ñ���PID��X\Y������һ����
	}
}

static void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)                                                         
{	
	static u16 altholdCount = 0;
	
	// Roll and Pitch
	attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
	attitude->roll = 0.15f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);
	
	// Thrust
	float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);
	
	*thrust = constrainf(thrustRaw + THRUST_BASE, 1000, 60000);	/*�����޷�*/
	
	thrustLpf += (*thrust - thrustLpf) * 0.003f;
	
	if(getCommanderKeyFlight())	/*���߷���״̬*/
	{
		if(fabs(state->acc.z) < 35.f)
		{
			altholdCount++;
			if(altholdCount > 1000)
			{
				altholdCount = 0;
				if(fabs(configParam.thrustBase - thrustLpf) > 1000.f)	/*���»�������ֵ*/
					configParam.thrustBase = thrustLpf;
			}
		}else
		{
			altholdCount = 0;
		}
	}else if(getCommanderKeyland() == false)	/*������ɣ���������*/
	{
		*thrust = 0;
	}
}

void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, float dt)                                                
{	
	if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs)
	{
		setpoint->velocity.x = 0.1f * pidUpdate(&pidX, setpoint->position.x - state->position.x);
		setpoint->velocity.y = 0.1f * pidUpdate(&pidY, setpoint->position.y - state->position.y);
	}
	
	if (setpoint->mode.z == modeAbs)
	{
		setpoint->velocity.z = pidUpdate(&pidZ, setpoint->position.z - state->position.z);
	}
	
	velocityController(thrust, attitude, setpoint, state);
}

/*��ȡ��������ֵ*/
float getAltholdThrust(void)
{
	return thrustLpf;
}

void positionResetAllPID(void)
{
	pidReset(&pidVX);
	pidReset(&pidVY);
	pidReset(&pidVZ);

	pidReset(&pidX);
	pidReset(&pidY);
	pidReset(&pidZ);
}
