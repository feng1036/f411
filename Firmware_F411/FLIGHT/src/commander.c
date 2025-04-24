#include <math.h>
#include "maths.h"
#include "commander.h"
#include "atkp.h"
#include "nvic.h"
#include "config_param.h"
#include "radiolink.h"
#include "remoter_ctrl.h"
#include "stabilizer.h"
#include "state_estimator.h"
#include "state_control.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

#define CLIMB_RATE			100.f
#define MAX_CLIMB_UP		100.f
#define MAX_CLIMB_DOWN		60.f

#define MIN_THRUST  	5000
#define MAX_THRUST  	60000

static bool isRCLocked;				/* ң������״̬ */
static ctrlValCache_t remoteCache;	/* ң�ػ������� */
static ctrlVal_t ctrlValLpf = {0.f};/* �������ݵ�ͨ */

static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 

static YawModeType yawMode = XMODE;	/* Ĭ��ΪX����ģʽ */
static commanderBits_t commander;

static void commanderLevelRPY(void)
{
	ctrlValLpf.roll = 0;
	ctrlValLpf.pitch = 0;
	ctrlValLpf.yaw = 0;
}

static void commanderDropToGround(void)
{
	commanderLevelRPY();
	ctrlValLpf.thrust = 0;
	if(commander.keyFlight)	/*���й����У�ң�����źŶϿ���һ������*/
	{
		commander.keyLand = true;
		commander.keyFlight = false;
	}	
}
u32 timestamp = 0;
/********************************************************
 *ctrlDataUpdate()	���¿�������
 *ң������ ���ȼ�����wifi��������
*********************************************************/
static void ctrlDataUpdate(void)	
{
	static float lpfVal = 0.2f;
	u32 tickNow = getSysTickCnt();	

	
	if ((tickNow - remoteCache.timestamp) < COMMANDER_WDT_TIMEOUT_STABILIZE) 
	{
		isRCLocked = false;			/*����*/
	}else 
	if ((tickNow - remoteCache.timestamp) < COMMANDER_WDT_TIMEOUT_SHUTDOWN) 
	{
		commanderLevelRPY();
	}else 
	{
		isRCLocked = true;			/*����*/
		commanderDropToGround();
	}
	
	if(isRCLocked == false)	/*����״̬*/
	{
		ctrlVal_t ctrlVal =  remoteCache.tarVal;	/*��ȡ����*/
		
		ctrlValLpf.thrust += (ctrlVal.thrust - ctrlValLpf.thrust) * lpfVal;
		ctrlValLpf.pitch += (ctrlVal.pitch - ctrlValLpf.pitch) * lpfVal;
		ctrlValLpf.roll += (ctrlVal.roll - ctrlValLpf.roll) * lpfVal;
		ctrlValLpf.yaw += (ctrlVal.yaw - ctrlValLpf.yaw) * lpfVal;
		
		configParam.trimP = ctrlVal.trimPitch;	/*����΢��ֵ*/
		configParam.trimR = ctrlVal.trimRoll;
		
		if (ctrlValLpf.thrust < MIN_THRUST)
			ctrlValLpf.thrust = 0;	
		else 		
			ctrlValLpf.thrust = (ctrlValLpf.thrust>=MAX_THRUST) ? MAX_THRUST:ctrlValLpf.thrust;
	}
}

/************************************************************************
* ����carefree(��ͷģʽ)���ο���������ϵ��������Χ��YAW��ת��
* ����ǰ����Ȼ���ֿ�ʼ�ķ������ģʽ�����ַǳ�ʵ��
************************************************************************/
static void rotateYawCarefree(setpoint_t *setpoint, const state_t *state)
{
	float yawRad = state->attitude.yaw * DEG2RAD;
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	
	if(setpoint->mode.x ==  modeDisable || setpoint->mode.y ==  modeDisable)	/*�ֶ��Ͷ���ģʽ*/
	{
		float originalRoll = setpoint->attitude.roll;
		float originalPitch = setpoint->attitude.pitch;

		setpoint->attitude.roll = originalRoll * cosy + originalPitch * siny;
		setpoint->attitude.pitch = originalPitch * cosy - originalRoll * siny;
	}
	else if(setpoint->mode.x ==  modeVelocity || setpoint->mode.y ==  modeVelocity)	/*����ģʽ*/
	{
		float originalVy = setpoint->velocity.y;
		float originalVx = setpoint->velocity.x;

		setpoint->velocity.y = originalVy * cosy + originalVx * siny;
		setpoint->velocity.x = originalVx * cosy - originalVy * siny;
	}
}

/*�ɿ����ݻ���*/
void flightCtrldataCache(ctrlSrc_e ctrlSrc, ctrlVal_t pk)
{
	remoteCache.tarVal = pk;
	remoteCache.timestamp = getSysTickCnt();
}//remoter_ctrl


/********************************************************
* flyerAutoLand()
* �����Զ�����
*********************************************************/
void flyerAutoLand(setpoint_t *setpoint,const state_t *state)
{	
	static u8 lowThrustCnt = 0;
	static float stateVelLpf  = -30.f;
	
	setpoint->mode.z = modeVelocity;
	stateVelLpf += (state->velocity.z -  stateVelLpf) * 0.1f;	/*���ʵ�ͨ*/
	setpoint->velocity.z = -70.f - stateVelLpf;	/*�����ٶ� ��λcm/s*/

	if(getAltholdThrust() < 20000.f)	/*��������ֵ�ϵ�*/
	{
		lowThrustCnt++;
		if(lowThrustCnt > 10)
		{
			lowThrustCnt = 0;
			commander.keyLand = false;
			commander.keyFlight = false;
			estRstAll();	/*��λ����*/
		}	
	}else
	{
		lowThrustCnt = 0;
	}
	
	float accZ = state->acc.z;
	if(minAccZ > accZ)
		minAccZ = accZ;
	if(maxAccZ < accZ)
		maxAccZ = accZ;

	
	if (minAccZ < -80.f && maxAccZ > 320.f)
	{
		commander.keyLand = false;
		commander.keyFlight = false;
		estRstAll();	/*��λ����*/
	}	
}

static bool initHigh = false;
static bool isAdjustingPosZ = false;/*����Zλ��*/
 static float errorPosZ = 0.f;		/*Zλ�����*/


void commanderGetSetpoint(setpoint_t *setpoint, state_t *state)
{	
	static float maxAccZ = 0.f;
	
	ctrlDataUpdate();	/*���¿�������*/
	
	state->isRCLocked = isRCLocked;	/*����ң��������״̬*/
	
	if(commander.ctrlMode & 0x01)/*����ģʽ*/
	{
		if(commander.keyLand)/*һ������*/
		{
			flyerAutoLand(setpoint, state);
		}
		else if(commander.keyFlight)/*һ�����*/ 
		{	
			setpoint->thrust = 0;
			setpoint->mode.z = modeAbs;		
			
			if (initHigh == false)
			{
				initHigh = true;	
				errorPosZ = 0.f;

				setFastAdjustPosParam(0, 1, 80.f);	/*һ����ɸ߶�80cm*/															
			}		
				
			float climb = ((ctrlValLpf.thrust - 32767.f) / 32767.f);
			if(climb > 0.f) 
				climb *= MAX_CLIMB_UP;
			else
				climb *= MAX_CLIMB_DOWN;
			
			if (fabsf(climb) > 5.f)
			{
				isAdjustingPosZ = true;												
				setpoint->mode.z = modeVelocity;
				setpoint->velocity.z = climb;

				if(climb < -(CLIMB_RATE/5.f))	/*������������*/
				{
					if(maxAccZ < state->acc.z)
						maxAccZ = state->acc.z;
					if(maxAccZ > 250.f)		/*�����������󣬷ɻ�����ͣ��*/
					{
						commander.keyFlight = false;
						estRstAll();	/*��λ����*/
					}
				}else
				{
					maxAccZ = 0.f;
				}
			}
			else if (isAdjustingPosZ == true)
			{
				isAdjustingPosZ = false;
			
				setpoint->mode.z = modeAbs;
				setpoint->position.z = state->position.z + errorPosZ;	/*������λ��*/									
			}
			else if(isAdjustingPosZ == false)	/*Zλ�����*/
			{
				errorPosZ = setpoint->position.z - state->position.z;
				errorPosZ = constrainf(errorPosZ, -10.f, 10.f);	/*����޷� ��λcm*/
			}			
		}
		else/*��½״̬*/
		{
			setpoint->mode.z = modeDisable;
			setpoint->thrust = 0;
			setpoint->velocity.z = 0;
			setpoint->position.z = 0;
			initHigh = false;
			isAdjustingPosZ = false;
		}
	}
	else /*�ֶ���ģʽ*/
	{
		setpoint->mode.z = modeDisable;
		setpoint->thrust = ctrlValLpf.thrust;
	}	
 	
	setpoint->attitude.roll = ctrlValLpf.roll;
	setpoint->attitude.pitch = ctrlValLpf.pitch;
	setpoint->attitude.yaw  = -ctrlValLpf.yaw;	/*ҡ�˷����yaw�����෴*/
	
		setpoint->mode.x = modeDisable;
		setpoint->mode.y = modeDisable;		
	
	setpoint->mode.roll = modeDisable;	
	setpoint->mode.pitch = modeDisable;	
	
	if(commander.flightMode)/*��ͷģʽ*/
	{
		yawMode = CAREFREE;		
		rotateYawCarefree(setpoint, state);
	}		
	else	/*X����ģʽ*/
	{
		yawMode = XMODE;
	}		
}//stabilizer.c

/* ��ȡ������΢��ֵ */
// void getAndUpdateTrim(float* pitch, float* roll)
// {
// 	*pitch = remoteCache.tarVal.trimPitch;
// 	*roll = remoteCache.tarVal.trimRoll;
// }

void setCommanderCtrlMode(u8 set)
{
	commander.ctrlMode = (set & 0x03);
}//remoter_ctrl.c
u8 getCommanderCtrlMode(void)
{
	return (commander.ctrlMode & 0x03);
}//anomal_detec.c

// u8 getCommanderFlightMode(void)
// {
// 	return (yawMode & 0x01);
// }

void setCommanderKeyFlight(bool set)
{
	commander.keyFlight = set;
	if(set == true)	/*һ����ɣ����������Сֵ*/
	{
		minAccZ = 0.f;
		maxAccZ = 0.f;
	}
}//remoter_ctrl.c + anomal_detec.c
bool getCommanderKeyFlight(void)
{
	return commander.keyFlight;
}//anomal_detec.c + remoter_ctrl.c + state_control.c

void setCommanderKeyland(bool set)
{
	commander.keyLand = set;
}//remoter_ctrl.c + anomal_detec.c
bool getCommanderKeyland(void)
{
	return commander.keyLand;
}//state_control.c + state_estimat.c

void setCommanderFlightmode(bool set)
{
	commander.flightMode = set;
}//remoter_ctrl.c

// void setCommanderEmerStop(bool set)
// {
// 	commander.emerStop = set;
// }
