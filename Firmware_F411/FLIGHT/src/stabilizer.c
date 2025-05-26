#include "system.h"
#include "stabilizer.h"
// #include "maths.h"
#include "communicate.h"
#include "nvic.h"
// #include "config_param.h"
#include <stdbool.h>
#include "sensors.h"
#include "string.h"
#include <math.h>
#include "ledseq.h"
// #include "motors.h"
#include "axis.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

#define DEG2RAD 0.017453293f
#define RAD2DEG 57.29578f

static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

/*�ǶȻ������޷�*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT 30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT 30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT 180.0

/*���ٶȻ������޷�*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT 500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT 500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT 50.0

PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

#define THRUST_BASE (20000) /*��������ֵ*/

#define PIDVX_OUTPUT_LIMIT 120.0f  // ROLL�޷�	(��λ���0.15��ϵ��)
#define PIDVY_OUTPUT_LIMIT 120.0f  // PITCH�޷�	(��λ���0.15��ϵ��)
#define PIDVZ_OUTPUT_LIMIT (40000) /*PID VZ�޷�*/

#define PIDX_OUTPUT_LIMIT 1200.0f // X���ٶ��޷�(��λcm/s ��0.1��ϵ��)
#define PIDY_OUTPUT_LIMIT 1200.0f // Y���ٶ��޷�(��λcm/s ��0.1��ϵ��)
#define PIDZ_OUTPUT_LIMIT 120.0f  // Z���ٶ��޷�(��λcm/s)

static float thrustLpf = THRUST_BASE; /*���ŵ�ͨ*/

PidObject pidVX;
PidObject pidVY;
PidObject pidVZ;

PidObject pidX;
PidObject pidY;
PidObject pidZ;

#define CLIMB_RATE 100.f
#define MAX_CLIMB_UP 100.f
#define MAX_CLIMB_DOWN 60.f

#define MIN_THRUST 5000
#define MAX_THRUST 60000

static bool isRCLocked;				 /* ң������״̬ */
static ctrlValCache_t remoteCache;	 /* ң�ػ������� */
static ctrlVal_t ctrlValLpf = {0.f}; /* �������ݵ�ͨ */

static float minAccZ = 0.f;
static float maxAccZ = 0.f;

static YawModeType yawMode = XMODE; /* Ĭ��ΪX����ģʽ */
static commanderBits_t commander;

static bool motorSetEnable = false;
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet = {0, 0, 0, 0};

#define ACCZ_SAMPLE 350

float Kp = 0.4f;   /*��������*/
float Ki = 0.001f; /*��������*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f; /*��������ۼ�*/

static float q0 = 1.0f; /*��Ԫ��*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float rMat[3][3]; /*��ת����*/

static float maxError = 0.f;				/*������*/
bool isGravityCalibrated = false;			/*�Ƿ�УУ׼���*/
static float baseAcc[3] = {0.f, 0.f, 1.0f}; /*��̬���ٶ�*/

#define ACC_LIMIT (1000.f)		   /*���ٶ��޷� ��λcm/s/s*/
#define ACC_LIMIT_MAX (1800.f)	   /*�����ٶ��޷� ��λcm/s/s*/
#define VELOCITY_LIMIT (130.f)	   /*�ٶ��޷� ��λcm/s*/
#define VELOCITY_LIMIT_MAX (500.f) /*����ٶ��޷� ��λcm/s*/

#define GRAVITY_CMSS (980.f)								  /*�������ٶ� ��λcm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE (GRAVITY_CMSS * 0.25f) // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway

static float wBaro = 0.35f;	   /*��ѹУ��Ȩ��*/
static float wAccBias = 0.01f; /*���ٶ�У��Ȩ��*/

static bool isRstHeight = false; /*��λ�߶�*/
static bool isRstAll = true;	 /*��λ����*/

static float fusedHeight;		   /*�ںϸ߶ȣ���ɵ�Ϊ0*/
static float fusedHeightLpf = 0.f; /*�ںϸ߶ȣ���ͨ*/
static float startBaroAsl = 0.f;   /*��ɵ㺣��*/

/*����ϵͳ*/
static estimator_t estimator =
	{
		.vAccDeadband = 4.0f,
		.accBias[0] = 0.0f,
		.accBias[1] = 0.0f,
		.accBias[2] = 0.0f,
		.acc[0] = 0.0f,
		.acc[1] = 0.0f,
		.acc[2] = 0.0f,
		.vel[0] = 0.0f,
		.vel[1] = 0.0f,
		.vel[2] = 0.0f,
		.pos[0] = 0.0f,
		.pos[1] = 0.0f,
		.pos[2] = 0.0f,
};

static bool isInit;

static setpoint_t setpoint;		/*����Ŀ��״̬*/
static sensorData_t sensorData; /*����������*/
static RemoteData_t pk;			/*��ͨ��ģ�鴫����������*/
static state_t state;			/*������̬*/
static control_t control;		/*������Ʋ���*/

static u16 velModeTimes = 0;  /*����ģʽ����*/
static u16 absModeTimes = 0;  /*����ֵģʽ����*/
static float setHeight = 0.f; /*�趨Ŀ��߶� ��λcm*/
static float baroLast = 0.f;
static float baroVelLpf = 0.f;

static ctrlVal_t remoterCtrl; /*���͵�commander��̬��������*/

configParam_t configParam =
	{
		.pidAngle = /*�Ƕ�PID*/
		{
			.roll =
				{
					.kp = 8.0,
					.ki = 0.0,
					.kd = 0.0,
				},
			.pitch =
				{
					.kp = 8.0,
					.ki = 0.0,
					.kd = 0.0,
				},
			.yaw =
				{
					.kp = 20.0,
					.ki = 0.0,
					.kd = 1.5,
				},
		},
		.pidRate = /*���ٶ�PID*/
		{
			.roll =
				{
					.kp = 300.0,
					.ki = 0.0,
					.kd = 6.5,
				},
			.pitch =
				{
					.kp = 300.0,
					.ki = 0.0,
					.kd = 6.5,
				},
			.yaw =
				{
					.kp = 200.0,
					.ki = 18.5,
					.kd = 0.0,
				},
		},
		.pidPos = /*λ��PID*/
		{
			.vx =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
				},
			.vy =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
				},
			.vz =
				{
					.kp = 100.0,
					.ki = 150.0,
					.kd = 10.0,
				},

			.x =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
				},
			.y =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
				},
			.z =
				{
					.kp = 6.0,
					.ki = 0.0,
					.kd = 4.5,
				},
		},

		.trimP = 0.f,		 /*pitch΢��*/
		.trimR = 0.f,		 /*roll΢��*/
		.thrustBase = 34000, /*�������Ż���ֵ*/
};

void stabilizerTask(void *param);

void stabilizerInit(void)
{
	if (isInit)
		return;

	stateControlInit(); /*��̬PID��ʼ��*/
	powerControlInit(); /*�����ʼ��*/

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;
	pass &= powerControlTest();
	return pass;
}

/*���ÿ��ٵ�������*/
void setFastAdjustPosParam(u16 velTimes, u16 absTimes, float height)
{
	if (velTimes != 0 && velModeTimes == 0)
	{
		baroLast = sensorData.baro.asl;
		baroVelLpf = 0.f;

		velModeTimes = velTimes;
	}
	if (absTimes != 0 && absModeTimes == 0)
	{
		setHeight = height;
		absModeTimes = absTimes;
	}
}

/*���ٵ����߶�*/
static void fastAdjustPosZ(void)
{
	if (velModeTimes > 0)
	{
		velModeTimes--;
		estRstHeight(); /*��λ����߶�*/

		float baroVel = (sensorData.baro.asl - baroLast) / 0.004f; /*250Hz*/
		baroLast = sensorData.baro.asl;
		baroVelLpf += (baroVel - baroVelLpf) * 0.35f;

		setpoint.mode.z = modeVelocity;
		state.velocity.z = baroVelLpf; /*��ѹ���ں�*/
		setpoint.velocity.z = -1.0f * baroVelLpf;
	}
	else if (absModeTimes > 0)
	{
		absModeTimes--;
		estRstAll(); /*��λ����*/
		setpoint.mode.z = modeAbs;
		setpoint.position.z = setHeight;
	}
}

void stabilizerDataprocess(RemoteData_t *anlPacket)
{
	if (anlPacket->msgID == DOWN_REMOTER)
	{
		remoterCtrlProcess(anlPacket); /*ң�������ݴ���*/
	}
	else if (anlPacket->msgID == DOWN_PID1 || anlPacket->msgID == DOWN_PID2)
	{
		attitudeDataprocess(anlPacket); /*PID��������*/
	}
	else if (anlPacket->msgID == DOWN_PID3 || anlPacket->msgID == DOWN_PID4)
	{
		positionDataprocess(anlPacket); /*PID��������*/
	}
	else if (anlPacket->msgID == DOWN_PID6)
	{
		powerDataprocess(anlPacket); /*�����������*/
	}
}

void stabilizerTask(void *param)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();

	ledseqRun(SYS_LED, seq_alive);

	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT); /*1ms������ʱ*/

		// ��ȡ6�����ѹ���ݣ�500Hz��
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			sensor_data_Read(&sensorData);
			/*��ȡ6�����ѹ����*/
		}

		// ��ȡͨ��ģ�鷢�͵�ң������,pid����,��Դ����,Ŀ����̬�ͷ���ģʽ�趨��250Hz��
		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			if (atkp_read(&pk)) /*��������*/
			{
				stabilizerDataprocess(&pk); /*ң������,pid����,��Դ���ݴ���*/
			}
		}

		// ��Ԫ����ŷ���Ǽ��㣨250Hz��
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
		}

		// λ��Ԥ�����㣨250Hz��
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{
			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
		}

		if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && getIsCalibrated() == true)
		{
			commanderGetSetpoint(&setpoint, &state); /*Ŀ�����ݺͷ���ģʽ�趨*/
		}

		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			fastAdjustPosZ(); /*���ٵ����߶�*/
		}
		/*�쳣���*/
		anomalDetec(&sensorData, &state, &control);

		/*PID����*/

		stateControl(&control, &sensorData, &state, &setpoint, tick);

		// ���Ƶ�������500Hz��
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			powerControl(&control);
		}

		tick++;
	}
}

void getAttitudeData(attitude_t *get)
{
	get->pitch = -state.attitude.pitch;
	get->roll = state.attitude.roll;
	get->yaw = -state.attitude.yaw;
}

float getBaroData(void)
{
	return sensorData.baro.asl;
}

void getSensorData(sensorData_t *get)
{
	*get = sensorData;
}

void getStateData(Axis3f *acc, Axis3f *vel, Axis3f *pos)
{
	acc->x = 1.0f * state.acc.x;
	acc->y = 1.0f * state.acc.y;
	acc->z = 1.0f * state.acc.z;
	vel->x = 1.0f * state.velocity.x;
	vel->y = 1.0f * state.velocity.y;
	vel->z = 1.0f * state.velocity.z;
	pos->x = 1.0f * state.position.x;
	pos->y = 1.0f * state.position.y;
	pos->z = 1.0f * state.position.z;
}

// remoter_ctrl.c

/*ң�����ݽ��մ���*/
void remoterCtrlProcess(RemoteData_t *pk)
{
	if (pk->data[0] == REMOTER_CMD)
	{
		switch (pk->data[1])
		{
		case CMD_FLIGHT_LAND:
			if (getCommanderKeyFlight() != true)
			{
				setCommanderKeyFlight(true);
				setCommanderKeyland(false);
			}
			else
			{
				setCommanderKeyFlight(false);
				setCommanderKeyland(true);
			}
			break;

		case CMD_EMER_STOP:
			setCommanderKeyFlight(false);
			setCommanderKeyland(false);
			break;
		}
	}
	else if (pk->data[0] == REMOTER_DATA)
	{
		remoterData_t remoterData = *(remoterData_t *)(pk->data + 1);

		remoterCtrl.roll = remoterData.roll;
		remoterCtrl.pitch = remoterData.pitch;
		remoterCtrl.yaw = remoterData.yaw;
		remoterCtrl.thrust = remoterData.thrust * 655.35f;
		remoterCtrl.trimPitch = remoterData.trimPitch;
		remoterCtrl.trimRoll = remoterData.trimRoll;

		setCommanderCtrlMode(remoterData.ctrlMode);
		setCommanderFlightmode(remoterData.flightMode);
		flightCtrldataCache(ATK_REMOTER, remoterCtrl);
	}
}

// anomal_detec.c

#if defined(DETEC_ENABLED)

static u16 outFlipCnt = 0;

static bool detecFreeFall(float accZ, float accMAG) /*����������*/
{
	static u16 cnt;

	/*��������*/
	if (fabs(accMAG) < DETEC_FF_THRESHOLD && fabs(accZ + 1.f) < DETEC_FF_THRESHOLD)
	{
		if (++cnt >= (DETEC_FF_COUNT))
		{
			return true;
		}
	}
	else
	{
		cnt = 0;
	}

	return false;
}

static bool detecTumbled(const state_t *state) /*��ײ���*/
{
	static u16 cnt;

	float fAbsRoll = fabs(state->attitude.roll);
	float fAbsPitch = fabs(state->attitude.pitch);
	float fMax = (fAbsRoll >= fAbsPitch) ? fAbsRoll : fAbsPitch;

	if (fMax > DETEC_TU_THRESHOLD)
	{
		if (++cnt >= DETEC_TU_COUNT)
		{
			return true;
		}
	}
	else
	{
		cnt = 0;
	}

	return false;
}
#endif

/*�쳣���*/
void anomalDetec(const sensorData_t *sensorData, const state_t *state, const control_t *control)
{
#if defined(DETEC_ENABLED)

	if (control->flipDir != CENTER)
	{
		outFlipCnt = 1000;
		return;
	}

	if (state->isRCLocked == false &&			 // ң��������״̬
		getCommanderKeyFlight() == false &&		 // δ����״̬
		(getCommanderCtrlMode() & 0x01) == 0x01) // ����ģʽ
	{
		float accMAG = (sensorData->acc.x * sensorData->acc.x) +
					   (sensorData->acc.y * sensorData->acc.y) +
					   (sensorData->acc.z * sensorData->acc.z);

		if (detecFreeFall(state->acc.z / 980.f, accMAG) == true) /*����������*/
		{
			setCommanderKeyFlight(true);
			setFastAdjustPosParam(35, 10, 0.f); /*���ÿ��ٵ���λ�ò���*/
		}
	}

	if (outFlipCnt > 0)
	{
		outFlipCnt--;
	}
	if (outFlipCnt == 0 && detecTumbled(state) == true) /*��ײ���*/
	{
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}

#endif
}

// state_estimator.c

/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
	estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt / 2.0f;
	estimator.vel[axis] += acc * dt;
}
/*λ��У��*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
	float ewdt = e * w * dt;
	estimator.pos[axis] += ewdt;
	estimator.vel[axis] += w * ewdt;
}
/*�ٶ�У��*/
// static void inavFilterCorrectVel(int axis, float dt, float e, float w)
//{
//    estimator.vel[axis] += e * w * dt;
// }

void positionEstimate(sensorData_t *sensorData, state_t *state, float dt)
{
	//	static float rangeLpf = 0.f;
	static float accLpf[3] = {0.f}; /*���ٶȵ�ͨ*/
	float weight = wBaro;

	float relateHight = sensorData->baro.asl - startBaroAsl; /*��ѹ��Ը߶�*/

	fusedHeight = relateHight; /*�ںϸ߶�*/

	fusedHeightLpf += (fusedHeight - fusedHeightLpf) * 0.1f; /*�ںϸ߶� ��ͨ*/

	if (isRstHeight)
	{
		isRstHeight = false;

		weight = 0.95f; /*����Ȩ�أ����ٵ���*/

		startBaroAsl = sensorData->baro.asl;

		estimator.pos[Z] = fusedHeight;
	}
	else if (isRstAll)
	{
		isRstAll = false;

		accLpf[Z] = 0.f;
		fusedHeight = 0.f;
		fusedHeightLpf = 0.f;
		startBaroAsl = sensorData->baro.asl;

		estimator.vel[Z] = 0.f;
		estimator.pos[Z] = fusedHeight;
	}

	Axis3f accelBF;

	accelBF.x = sensorData->acc.x * GRAVITY_CMSS - estimator.accBias[X];
	accelBF.y = sensorData->acc.y * GRAVITY_CMSS - estimator.accBias[Y];
	accelBF.z = sensorData->acc.z * GRAVITY_CMSS - estimator.accBias[Z];

	/* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
	imuTransformVectorBodyToEarth(&accelBF);

	estimator.acc[X] = applyDeadbandf(accelBF.x, estimator.vAccDeadband); /*ȥ�������ļ��ٶ�*/
	estimator.acc[Y] = applyDeadbandf(accelBF.y, estimator.vAccDeadband); /*ȥ�������ļ��ٶ�*/
	estimator.acc[Z] = applyDeadbandf(accelBF.z, estimator.vAccDeadband); /*ȥ�������ļ��ٶ�*/

	for (u8 i = 0; i < 3; i++)
		accLpf[i] += (estimator.acc[i] - accLpf[i]) * 0.1f; /*���ٶȵ�ͨ*/

	bool isKeyFlightLand = ((getCommanderKeyFlight() == true) || (getCommanderKeyland() == true)); /*���߷ɻ��߽���״̬*/

	if (isKeyFlightLand == true) /*���߷ɻ��߽���״̬*/
	{
		state->acc.x = constrainf(accLpf[X], -ACC_LIMIT, ACC_LIMIT); /*���ٶ��޷�*/
		state->acc.y = constrainf(accLpf[Y], -ACC_LIMIT, ACC_LIMIT); /*���ٶ��޷�*/
		state->acc.z = constrainf(accLpf[Z], -ACC_LIMIT, ACC_LIMIT); /*���ٶ��޷�*/
	}
	else
	{
		state->acc.x = constrainf(estimator.acc[X], -ACC_LIMIT_MAX, ACC_LIMIT_MAX); /*�����ٶ��޷�*/
		state->acc.y = constrainf(estimator.acc[Y], -ACC_LIMIT_MAX, ACC_LIMIT_MAX); /*�����ٶ��޷�*/
		state->acc.z = constrainf(estimator.acc[Z], -ACC_LIMIT_MAX, ACC_LIMIT_MAX); /*�����ٶ��޷�*/
	}

	float errPosZ = fusedHeight - estimator.pos[Z];

	/* λ��Ԥ��: Z-axis */
	inavFilterPredict(Z, dt, estimator.acc[Z]);
	/* λ��У��: Z-axis */
	inavFilterCorrectPos(Z, dt, errPosZ, weight);

	/*���ٶ�ƫ��У��*/
	Axis3f accelBiasCorr = {{0, 0, 0}};

	accelBiasCorr.z -= errPosZ * sq(wBaro);
	float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);
	if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE))
	{
		/* transform error vector from NEU frame to body frame */
		imuTransformVectorEarthToBody(&accelBiasCorr);

		/* Correct accel bias */
		estimator.accBias[X] += accelBiasCorr.x * wAccBias * dt;
		estimator.accBias[Y] += accelBiasCorr.y * wAccBias * dt;
		estimator.accBias[Z] += accelBiasCorr.z * wAccBias * dt;
	}

	if (isKeyFlightLand == true) /*���߷ɻ��߽���״̬*/
	{
		state->velocity.x = constrainf(estimator.vel[X], -VELOCITY_LIMIT, VELOCITY_LIMIT); /*�ٶ��޷� VELOCITY_LIMIT*/
		state->velocity.y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT, VELOCITY_LIMIT); /*�ٶ��޷� VELOCITY_LIMIT*/
		state->velocity.z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT, VELOCITY_LIMIT); /*�ٶ��޷� VELOCITY_LIMIT*/
	}
	else
	{
		state->velocity.x = constrainf(estimator.vel[X], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX); /*����ٶ��޷� VELOCITY_LIMIT_MAX*/
		state->velocity.y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX); /*����ٶ��޷� VELOCITY_LIMIT_MAX*/
		state->velocity.z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX); /*����ٶ��޷� VELOCITY_LIMIT_MAX*/
	}

	state->position.x = estimator.pos[X];
	state->position.y = estimator.pos[Y];
	state->position.z = estimator.pos[Z];
} // stabilizer.c

// /*��ȡ�ںϸ߶� ��λcm*/
// float getFusedHeight(void)
// {
// 	return fusedHeightLpf;
// }

/*��λ����߶�*/
void estRstHeight(void)
{
	isRstHeight = true;
} // stablilizer.c

/*��λ���й���*/
void estRstAll(void)
{
	isRstAll = true;
} // stabilizer.c + commander.c

// sensfusion6.c

static float invSqrt(float x); /*���ٿ�ƽ����*/

static void calBaseAcc(float *acc) /*���㾲̬���ٶ�*/
{
	static u16 cnt = 0;
	static float accZMin = 1.5;
	static float accZMax = 0.5;
	static float sumAcc[3] = {0.f};

	for (u8 i = 0; i < 3; i++)
		sumAcc[i] += acc[i];

	if (acc[2] < accZMin)
		accZMin = acc[2];
	if (acc[2] > accZMax)
		accZMax = acc[2];

	if (++cnt >= ACCZ_SAMPLE) /*��������*/
	{
		cnt = 0;
		maxError = accZMax - accZMin;
		accZMin = 1.5;
		accZMax = 0.5;

		if (maxError < 0.015f)
		{
			for (u8 i = 0; i < 3; i++)
				baseAcc[i] = sumAcc[i] / ACCZ_SAMPLE;

			isGravityCalibrated = true;

			ledseqRun(SYS_LED, seq_calibrated); /*У׼ͨ��ָʾ��*/
		}

		for (u8 i = 0; i < 3; i++)
			sumAcc[i] = 0.f;
	}
}

/*������ת����*/
void imuComputeRotationMatrix(void)
{
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;

	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q3 = q2 * q3;

	rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
	rMat[0][1] = 2.0f * (q1q2 + -q0q3);
	rMat[0][2] = 2.0f * (q1q3 - -q0q2);

	rMat[1][0] = 2.0f * (q1q2 - -q0q3);
	rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
	rMat[1][2] = 2.0f * (q2q3 + -q0q1);

	rMat[2][0] = 2.0f * (q1q3 + -q0q2);
	rMat[2][1] = 2.0f * (q2q3 - -q0q1);
	rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state, float dt) /*�����ں� �����˲�*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	Axis3f tempacc = acc;

	gyro.x = gyro.x * DEG2RAD; /* ��ת���� */
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;

	/* ���ٶȼ������Чʱ,���ü��ٶȼƲ���������*/
	if ((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		/*��λ�����ټƲ���ֵ*/
		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;

		/*���ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ����������˼���*/
		ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
		ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
		ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);

		/*����ۼƣ�����ֳ������*/
		exInt += Ki * ex * dt;
		eyInt += Ki * ey * dt;
		ezInt += Ki * ez * dt;

		/*�ò���������PI����������ƫ�����������ݶ����е�ƫ����*/
		gyro.x += Kp * ex + exInt;
		gyro.y += Kp * ey + eyInt;
		gyro.z += Kp * ez + ezInt;
	}
	/* һ�׽����㷨����Ԫ���˶�ѧ���̵���ɢ����ʽ�ͻ��� */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
	q1 += (q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
	q2 += (q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
	q3 += (q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;

	/*��λ����Ԫ��*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;

	imuComputeRotationMatrix(); /*������ת����*/

	/*����roll pitch yaw ŷ����*/
	state->attitude.pitch = -asinf(rMat[2][0]) * RAD2DEG;
	state->attitude.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	state->attitude.yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;

	if (!isGravityCalibrated) /*δУ׼*/
	{
		//		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
		//		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
		accBuf[2] = tempacc.x * rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2]; /*accz*/
		calBaseAcc(accBuf);																	  /*���㾲̬���ٶ�*/
	}
} // stablilizer.c

/*���嵽����*/
void imuTransformVectorBodyToEarth(Axis3f *v)
{
	/* From body frame to earth frame */
	const float x = rMat[0][0] * v->x + rMat[0][1] * v->y + rMat[0][2] * v->z;
	const float y = rMat[1][0] * v->x + rMat[1][1] * v->y + rMat[1][2] * v->z;
	const float z = rMat[2][0] * v->x + rMat[2][1] * v->y + rMat[2][2] * v->z;

	float yawRad = atan2f(rMat[1][0], rMat[0][0]);
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	float vx = x * cosy + y * siny;
	float vy = y * cosy - x * siny;

	v->x = vx;
	v->y = -vy;
	v->z = z - baseAcc[2] * 980.f; /*ȥ���������ٶ�*/
} // state_estimator.c

/*���򵽻���*/
void imuTransformVectorEarthToBody(Axis3f *v)
{
	v->y = -v->y;

	/* From earth frame to body frame */
	const float x = rMat[0][0] * v->x + rMat[1][0] * v->y + rMat[2][0] * v->z;
	const float y = rMat[0][1] * v->x + rMat[1][1] * v->y + rMat[2][1] * v->z;
	const float z = rMat[0][2] * v->x + rMat[1][2] * v->y + rMat[2][2] * v->z;

	v->x = x;
	v->y = y;
	v->z = z;
} // state_estimator.c

// Fast inverse square-root
float invSqrt(float x) /*���ٿ�ƽ����*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

bool getIsCalibrated(void)
{
	return isGravityCalibrated;
} // stabilizer.c

// power_control.c

void powerControlInit(void)
{
	motorsInit();
} // stabilizer.c

void powerDataprocess(RemoteData_t *anlPacket)
{
	s16 enable = ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
	s16 m1_set = ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
	s16 m2_set = ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
	s16 m3_set = ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
	s16 m4_set = ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
	setMotorPWM(enable, m1_set, m2_set, m3_set, m4_set);
} // stabilizer.c

bool powerControlTest(void)
{
	bool pass = true;

	pass &= motorsTest();

	return pass;
} // stabilizer.c

u16 limitThrust(int value)
{
	if (value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if (value < 0)
	{
		value = 0;
	}

	return (u16)value;
}

void powerControl(control_t *control) /*�����������*/
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
	motorsSetRatio(MOTOR_M1, motorPWM.m1); /*���Ƶ������ٷֱ�*/
	motorsSetRatio(MOTOR_M2, motorPWM.m2);
	motorsSetRatio(MOTOR_M3, motorPWM.m3);
	motorsSetRatio(MOTOR_M4, motorPWM.m4);
} // stabilizer.c

// void getMotorPWM(motorPWM_t* get)
// {
// 	*get = motorPWM;
// }

void setMotorPWM(bool enable, u32 m1_set, u32 m2_set, u32 m3_set, u32 m4_set)
{
	motorSetEnable = enable;
	motorPWMSet.m1 = m1_set;
	motorPWMSet.m2 = m2_set;
	motorPWMSet.m3 = m3_set;
	motorPWMSet.m4 = m4_set;
}

// commander.c

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
	if (commander.keyFlight) /*���й����У�ң�����źŶϿ���һ������*/
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
		isRCLocked = false; /*����*/
	}
	else if ((tickNow - remoteCache.timestamp) < COMMANDER_WDT_TIMEOUT_SHUTDOWN)
	{
		commanderLevelRPY();
	}
	else
	{
		isRCLocked = true; /*����*/
		commanderDropToGround();
	}

	if (isRCLocked == false) /*����״̬*/
	{
		ctrlVal_t ctrlVal = remoteCache.tarVal; /*��ȡ����*/

		ctrlValLpf.thrust += (ctrlVal.thrust - ctrlValLpf.thrust) * lpfVal;
		ctrlValLpf.pitch += (ctrlVal.pitch - ctrlValLpf.pitch) * lpfVal;
		ctrlValLpf.roll += (ctrlVal.roll - ctrlValLpf.roll) * lpfVal;
		ctrlValLpf.yaw += (ctrlVal.yaw - ctrlValLpf.yaw) * lpfVal;

		configParam.trimP = ctrlVal.trimPitch; /*����΢��ֵ*/
		configParam.trimR = ctrlVal.trimRoll;

		if (ctrlValLpf.thrust < MIN_THRUST)
			ctrlValLpf.thrust = 0;
		else
			ctrlValLpf.thrust = (ctrlValLpf.thrust >= MAX_THRUST) ? MAX_THRUST : ctrlValLpf.thrust;
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

	if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) /*�ֶ��Ͷ���ģʽ*/
	{
		float originalRoll = setpoint->attitude.roll;
		float originalPitch = setpoint->attitude.pitch;

		setpoint->attitude.roll = originalRoll * cosy + originalPitch * siny;
		setpoint->attitude.pitch = originalPitch * cosy - originalRoll * siny;
	}
	else if (setpoint->mode.x == modeVelocity || setpoint->mode.y == modeVelocity) /*����ģʽ*/
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
} // remoter_ctrl
// stavilier.c

/********************************************************
 * flyerAutoLand()
 * �����Զ�����
 *********************************************************/
void flyerAutoLand(setpoint_t *setpoint, const state_t *state)
{
	static u8 lowThrustCnt = 0;
	static float stateVelLpf = -30.f;

	setpoint->mode.z = modeVelocity;
	stateVelLpf += (state->velocity.z - stateVelLpf) * 0.1f; /*���ʵ�ͨ*/
	setpoint->velocity.z = -70.f - stateVelLpf;				 /*�����ٶ� ��λcm/s*/

	if (getAltholdThrust() < 20000.f) /*��������ֵ�ϵ�*/
	{
		lowThrustCnt++;
		if (lowThrustCnt > 10)
		{
			lowThrustCnt = 0;
			commander.keyLand = false;
			commander.keyFlight = false;
			estRstAll(); /*��λ����*/
		}
	}
	else
	{
		lowThrustCnt = 0;
	}

	float accZ = state->acc.z;
	if (minAccZ > accZ)
		minAccZ = accZ;
	if (maxAccZ < accZ)
		maxAccZ = accZ;

	if (minAccZ < -80.f && maxAccZ > 320.f)
	{
		commander.keyLand = false;
		commander.keyFlight = false;
		estRstAll(); /*��λ����*/
	}
}

static bool initHigh = false;
static bool isAdjustingPosZ = false; /*����Zλ��*/
static float errorPosZ = 0.f;		 /*Zλ�����*/

void commanderGetSetpoint(setpoint_t *setpoint, state_t *state)
{
	static float maxAccZ = 0.f;

	ctrlDataUpdate(); /*���¿�������*/

	state->isRCLocked = isRCLocked; /*����ң��������״̬*/

	if (commander.ctrlMode & 0x01) /*����ģʽ*/
	{
		if (commander.keyLand) /*һ������*/
		{
			flyerAutoLand(setpoint, state);
		}
		else if (commander.keyFlight) /*һ�����*/
		{
			setpoint->thrust = 0;
			setpoint->mode.z = modeAbs;

			if (initHigh == false)
			{
				initHigh = true;
				errorPosZ = 0.f;

				setFastAdjustPosParam(0, 1, 80.f); /*һ����ɸ߶�80cm*/
			}

			float climb = ((ctrlValLpf.thrust - 32767.f) / 32767.f);
			if (climb > 0.f)
				climb *= MAX_CLIMB_UP;
			else
				climb *= MAX_CLIMB_DOWN;

			if (fabsf(climb) > 5.f)
			{
				isAdjustingPosZ = true;
				setpoint->mode.z = modeVelocity;
				setpoint->velocity.z = climb;

				if (climb < -(CLIMB_RATE / 5.f)) /*������������*/
				{
					if (maxAccZ < state->acc.z)
						maxAccZ = state->acc.z;
					if (maxAccZ > 250.f) /*�����������󣬷ɻ�����ͣ��*/
					{
						commander.keyFlight = false;
						estRstAll(); /*��λ����*/
					}
				}
				else
				{
					maxAccZ = 0.f;
				}
			}
			else if (isAdjustingPosZ == true)
			{
				isAdjustingPosZ = false;

				setpoint->mode.z = modeAbs;
				setpoint->position.z = state->position.z + errorPosZ; /*������λ��*/
			}
			else if (isAdjustingPosZ == false) /*Zλ�����*/
			{
				errorPosZ = setpoint->position.z - state->position.z;
				errorPosZ = constrainf(errorPosZ, -10.f, 10.f); /*����޷� ��λcm*/
			}
		}
		else /*��½״̬*/
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
	setpoint->attitude.yaw = -ctrlValLpf.yaw; /*ҡ�˷����yaw�����෴*/

	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;

	setpoint->mode.roll = modeDisable;
	setpoint->mode.pitch = modeDisable;

	if (commander.flightMode) /*��ͷģʽ*/
	{
		yawMode = CAREFREE;
		rotateYawCarefree(setpoint, state);
	}
	else /*X����ģʽ*/
	{
		yawMode = XMODE;
	}
} // stabilizer.c

/* ��ȡ������΢��ֵ */
// void getAndUpdateTrim(float* pitch, float* roll)
// {
// 	*pitch = remoteCache.tarVal.trimPitch;
// 	*roll = remoteCache.tarVal.trimRoll;
// }

void setCommanderCtrlMode(u8 set)
{
	commander.ctrlMode = (set & 0x03);
} // remoter_ctrl.c
u8 getCommanderCtrlMode(void)
{
	return (commander.ctrlMode & 0x03);
} // anomal_detec.c

// u8 getCommanderFlightMode(void)
// {
// 	return (yawMode & 0x01);
// }

void setCommanderKeyFlight(bool set)
{
	commander.keyFlight = set;
	if (set == true) /*һ����ɣ����������Сֵ*/
	{
		minAccZ = 0.f;
		maxAccZ = 0.f;
	}
} // remoter_ctrl.c + anomal_detec.c
bool getCommanderKeyFlight(void)
{
	return commander.keyFlight;
} // anomal_detec.c + remoter_ctrl.c + state_control.c

void setCommanderKeyland(bool set)
{
	commander.keyLand = set;
} // remoter_ctrl.c + anomal_detec.c
bool getCommanderKeyland(void)
{
	return commander.keyLand;
} // state_control.c + state_estimat.c

void setCommanderFlightmode(bool set)
{
	commander.flightMode = set;
} // remoter_ctrl.c

// void setCommanderEmerStop(bool set)
// {
// 	commander.emerStop = set;
// }

// state_control.c

void stateControlInit(void)
{
	attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT);		   /*��ʼ����̬PID*/
	positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*��ʼ��λ��PID*/
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

	// �ǶȻ����⻷��
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

		if (control->flipDir == CENTER)
		{
			attitudeDesired.yaw += setpoint->attitude.yaw / ANGEL_PID_RATE; /*����YAW ����ģʽ*/
			if (attitudeDesired.yaw > 180.0f)
				attitudeDesired.yaw -= 360.0f;
			if (attitudeDesired.yaw < -180.0f)
				attitudeDesired.yaw += 360.0f;
		}

		attitudeDesired.roll += configParam.trimR; // ����΢��ֵ
		attitudeDesired.pitch += configParam.trimP;

		attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);
	}

	// ���ٶȻ����ڻ���
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

		attitudeResetAllPID();					   /*��λ��̬PID*/
		positionResetAllPID();					   /*��λλ��PID*/
		attitudeDesired.yaw = state->attitude.yaw; /*��λ���������yawֵ*/

		if (cnt++ > 1500)
		{
			cnt = 0;
		}
	}
	else
	{
		cnt = 0;
	}
}

// attitude_pid.c �е�����

static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
} // state_control

void attitudeControlInit(float ratePidDt, float anglePidDt)
{
	pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, anglePidDt);		/*roll  �Ƕ�PID��ʼ��*/
	pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, anglePidDt);		/*pitch �Ƕ�PID��ʼ��*/
	pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, anglePidDt);			/*yaw   �Ƕ�PID��ʼ��*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);	/*roll  �ǶȻ����޷�����*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT); /*pitch �ǶȻ����޷�����*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);		/*yaw   �ǶȻ����޷�����*/

	pidInit(&pidRateRoll, 0, configParam.pidRate.roll, ratePidDt);		  /*roll  ���ٶ�PID��ʼ��*/
	pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, ratePidDt);	  /*pitch ���ٶ�PID��ʼ��*/
	pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, ratePidDt);		  /*yaw   ���ٶ�PID��ʼ��*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);	  /*roll  ���ٶȻ����޷�����*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT); /*pitch ���ٶȻ����޷�����*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);	  /*yaw   ���ٶȻ����޷�����*/
} // state_control

void attitudeDataprocess(RemoteData_t *anlPacket)
{
	if (anlPacket->msgID == DOWN_PID1)
	{
		pidRateRoll.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
		pidRateRoll.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
		pidRateRoll.kd = 0.1 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
		pidRatePitch.kp = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
		pidRatePitch.ki = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
		pidRatePitch.kd = 0.1 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
		pidRateYaw.kp = 0.1 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
		pidRateYaw.ki = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
		pidRateYaw.kd = 0.1 * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));
	}
	else if (anlPacket->msgID == DOWN_PID2)
	{
		pidAngleRoll.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
		pidAngleRoll.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
		pidAngleRoll.kd = 0.1 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
		pidAnglePitch.kp = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
		pidAnglePitch.ki = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
		pidAnglePitch.kd = 0.1 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
		pidAngleYaw.kp = 0.1 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
		pidAngleYaw.ki = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
		pidAngleYaw.kd = 0.1 * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));
	}
} // stabilizer

void attitudeRatePID(Axis3f *actualRate, attitude_t *desiredRate, control_t *output) /* ���ٶȻ�PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->x));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
} // state_control

void attitudeAnglePID(attitude_t *actualAngle, attitude_t *desiredAngle, attitude_t *outDesiredRate) /* �ǶȻ�PID */
{
	outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

	float yawError = desiredAngle->yaw - actualAngle->yaw;
	if (yawError > 180.0f)
		yawError -= 360.0f;
	else if (yawError < -180.0)
		yawError += 360.0f;
	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
} // state_control

void attitudeControllerResetRollAttitudePID(void)
{
	pidReset(&pidAngleRoll);
} // state_control

void attitudeControllerResetPitchAttitudePID(void)
{
	pidReset(&pidAnglePitch);
} // state_control

void attitudeResetAllPID(void) /*��λPID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
} // state_control

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
// }//��attitude_pid.c�У�û���õ�

// position_pid

void positionControlInit(float velocityPidDt, float posPidDt)
{
	pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt); /*vx PID��ʼ��*/
	pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt); /*vy PID��ʼ��*/
	pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt); /*vz PID��ʼ��*/
	pidSetOutputLimit(&pidVX, PIDVX_OUTPUT_LIMIT);			  /* ����޷� */
	pidSetOutputLimit(&pidVY, PIDVY_OUTPUT_LIMIT);			  /* ����޷� */
	pidSetOutputLimit(&pidVZ, PIDVZ_OUTPUT_LIMIT);			  /* ����޷� */

	pidInit(&pidX, 0, configParam.pidPos.x, posPidDt); /*x PID��ʼ��*/
	pidInit(&pidY, 0, configParam.pidPos.y, posPidDt); /*y PID��ʼ��*/
	pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt); /*z PID��ʼ��*/
	pidSetOutputLimit(&pidX, PIDX_OUTPUT_LIMIT);	   /* ����޷� */
	pidSetOutputLimit(&pidY, PIDY_OUTPUT_LIMIT);	   /* ����޷� */
	pidSetOutputLimit(&pidZ, PIDZ_OUTPUT_LIMIT);	   /* ����޷� */
} // state_control

void positionDataprocess(RemoteData_t *anlPacket)
{
	if (anlPacket->msgID == DOWN_PID3)
	{
		pidVZ.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
		pidVZ.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
		pidVZ.kd = 0.1 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));

		pidZ.kp = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
		pidZ.ki = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
		pidZ.kd = 0.1 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));

		pidVX.kp = 0.1 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
		pidVX.ki = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
		pidVX.kd = 0.1 * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

		pidVY = pidVX; // λ������PID��X\Y������һ����
	}
	else if (anlPacket->msgID == DOWN_PID4)
	{
		pidX.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
		pidX.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
		pidX.kd = 0.1 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));

		pidY = pidX; // λ�ñ���PID��X\Y������һ����
	}
} // stabilizer

static void velocityController(float *thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)
{
	static u16 altholdCount = 0;

	// Roll and Pitch
	attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
	attitude->roll = 0.15f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);

	// Thrust
	float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);

	*thrust = constrainf(thrustRaw + THRUST_BASE, 1000, 60000); /*�����޷�*/

	thrustLpf += (*thrust - thrustLpf) * 0.003f;

	if (getCommanderKeyFlight()) /*���߷���״̬*/
	{
		if (fabs(state->acc.z) < 35.f)
		{
			altholdCount++;
			if (altholdCount > 1000)
			{
				altholdCount = 0;
				if (fabs(configParam.thrustBase - thrustLpf) > 1000.f) /*���»�������ֵ*/
					configParam.thrustBase = thrustLpf;
			}
		}
		else
		{
			altholdCount = 0;
		}
	}
	else if (getCommanderKeyland() == false) /*������ɣ���������*/
	{
		*thrust = 0;
	}
} // state_control

void positionController(float *thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, float dt)
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
} // state_control

/*��ȡ��������ֵ*/
float getAltholdThrust(void)
{
	return thrustLpf;
} // commander.c

void positionResetAllPID(void)
{
	pidReset(&pidVX);
	pidReset(&pidVY);
	pidReset(&pidVZ);

	pidReset(&pidX);
	pidReset(&pidY);
	pidReset(&pidZ);
}

// pid.c

void pidInit(PidObject *pid, const float desired, const pidInit_t pidParam, const float dt)
{
	pid->error = 0;
	pid->prevError = 0;
	pid->integ = 0;
	pid->deriv = 0;
	pid->desired = desired;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
	pid->outputLimit = DEFAULT_PID_OUTPUT_LIMIT;
	pid->dt = dt;
} // state_control

float pidUpdate(PidObject *pid, const float error)
{
	float output;

	pid->error = error;

	pid->integ += pid->error * pid->dt;

	// �����޷�
	if (pid->integ > pid->iLimit)
	{
		pid->integ = pid->iLimit;
	}
	else if (pid->integ < -pid->iLimit)
	{
		pid->integ = -pid->iLimit;
	}

	pid->deriv = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	output = pid->outP + pid->outI + pid->outD;

	// ����޷�
	if (pid->outputLimit != 0)
	{
		if (output > pid->outputLimit)
			output = pid->outputLimit;
		else if (output < -pid->outputLimit)
			output = -pid->outputLimit;
	}

	pid->prevError = pid->error;

	pid->out = output;
	return output;
} // state_control

void pidSetIntegralLimit(PidObject *pid, const float limit)
{
	pid->iLimit = limit;
} // state_control

void pidSetOutputLimit(PidObject *pid, const float limit)
{
	pid->outputLimit = limit;
} // state_control

void pidReset(PidObject *pid)
{
	pid->error = 0;
	pid->prevError = 0;
	pid->integ = 0;
	pid->deriv = 0;
} // state_control

float applyDeadbandf(float value, float deadband)
{
	if (ABS(value) < deadband)
	{
		value = 0;
	}
	else if (value > 0)
	{
		value -= deadband;
	}
	else if (value < 0)
	{
		value += deadband;
	}
	return value;
}

float constrainf(float amt, float low, float high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}