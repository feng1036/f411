/******************************************************************************
Filename    : stabilizer.h
Author      : fxy
Date        : 28/05/2025
Licence     : The Unlicense; see LICENSE for details.
Description : The header file for the stabilizer.
******************************************************************************/

/* Define ********************************************************************/
#ifndef __STABALIZER_H
#define __STABALIZER_H
/*****************************************************************************/

/* Include *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
/* End Include ***************************************************************/


#define DEFAULT_PID_INTEGRATION_LIMIT 		500.0 //Ĭ��pid�Ļ����޷�
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0	  //Ĭ��pid����޷���0Ϊ���޷�
#define DETEC_ENABLED
#define DETEC_FF_THRESHOLD 					0.05f	/* accZ�ӽ�-1.0�̶� ��ʾFree Fall */
#define DETEC_FF_COUNT 						50  	/* ������������� 1000Hz�������� */
#define DETEC_TU_THRESHOLD 					60		/* ��ײ�����ֵ60��*/
#define DETEC_TU_COUNT 						100  	/* ��ײ������ 1000Hz�������� */
#define CMD_GET_CANFLY						0x02	/*��ȡ�����Ƿ��ܷ�*/
#define CMD_FLIGHT_LAND						0x03	/*��ɡ�����*/
#define CMD_EMER_STOP						0x04	/*����ͣ��*/
#define ACK_MSG								0x01
#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#define MAIN_LOOP_RATE 			RATE_1000_HZ
#define MAIN_LOOP_DT			(uint32_t)(1000/MAIN_LOOP_RATE)	/*��λms*/

#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//��̬��������
#define ATTITUDE_ESTIMAT_DT		(1.0/RATE_250_HZ)

#define POSITION_ESTIMAT_RATE	RATE_250_HZ	//λ��Ԥ������
#define POSITION_ESTIMAT_DT		(1.0/RATE_250_HZ)

#define RATE_PID_RATE			RATE_500_HZ //���ٶȻ����ڻ���PID����
#define RATE_PID_DT				(1.0/RATE_500_HZ)

#define ANGEL_PID_RATE			ATTITUDE_ESTIMAT_RATE //�ǶȻ����⻷��PID����
#define ANGEL_PID_DT			(1.0/ATTITUDE_ESTIMAT_RATE)

#define VELOCITY_PID_RATE		POSITION_ESTIMAT_RATE //�ٶȻ����ڻ���PID����
#define VELOCITY_PID_DT			(1.0/POSITION_ESTIMAT_RATE)

#define POSITION_PID_RATE		POSITION_ESTIMAT_RATE //λ�û����⻷��PID����
#define POSITION_PID_DT			(1.0/POSITION_ESTIMAT_RATE)

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000

typedef struct  
{
	uint32_t timestamp;	/*ʱ���*/

	float roll;
	float pitch;
	float yaw;
} attitude_t;

struct  vec3_s 
{
	uint32_t timestamp;	/*ʱ���*/

	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s 
{
	uint32_t timestamp;

	union 
	{
		struct 
		{
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct 
		{
			float x;
			float y;
			float z;
			float w;
		};
	};
} quaternion_t;

typedef struct toaMeasurement_s 
{
	int8_t senderId;
	float x, y, z;
	int64_t rx, tx;
} toaMeasurement_t;

typedef struct tdoaMeasurement_s {
  point_t anchorPosition[2];
  float distanceDiff;
  float stdDev;
} tdoaMeasurement_t;

typedef struct positionMeasurement_s 
{
	union 
	{
		struct 
		{
			float x;
			float y;
			float z;
		};
		float pos[3];
	};
	float stdDev;
} positionMeasurement_t;

typedef struct distanceMeasurement_s 
{
	union 
	{
		struct 
		{
			float x;
			float y;
			float z;
		};
		float pos[3];
	};
	float distance;
	float stdDev;
} distanceMeasurement_t;

typedef struct zRange_s 
{
	uint32_t timestamp;	//ʱ���
	float distance;		//��������
	float quality;		//���Ŷ�
} zRange_t;

/** Flow measurement**/
typedef struct flowMeasurement_s 
{
	uint32_t timestamp;
	union 
	{
		struct 
		{
			float dpixelx;  // Accumulated pixel count x
			float dpixely;  // Accumulated pixel count y
		};
		float dpixel[2];  // Accumulated pixel count
	};
	float stdDevX;      // Measurement standard deviation
	float stdDevY;      // Measurement standard deviation
	float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;


/** TOF measurement**/
typedef struct tofMeasurement_s 
{
	uint32_t timestamp;
	float distance;
	float stdDev;
} tofMeasurement_t;


typedef struct
{
	attitude_t attitude;
	quaternion_t attitudeQuaternion;
	point_t position;
	velocity_t velocity;
	acc_t acc;
	bool isRCLocked;
} state_t;

enum dir_e
{
	CENTER=0,
	FORWARD,
	BACK,
	LEFT,
	RIGHT,
};

typedef struct
{
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	float thrust;
	enum dir_e flipDir;		/*��������*/
} control_t;

typedef enum
{
	modeDisable = 0,/*�ر�ģʽ*/
	modeAbs,		/*����ֵģʽ*/
	modeVelocity	/*����ģʽ*/
} mode_e;

typedef struct
{
	mode_e x;
	mode_e y;
	mode_e z;
	mode_e roll;
	mode_e pitch;
	mode_e yaw;
}mode_t;

typedef struct
{
	attitude_t attitude;		// deg	
	attitude_t attitudeRate;	// deg/s
	point_t position;         	// m
	velocity_t velocity;      	// m/s
	mode_t mode;
	float thrust;
} setpoint_t;

typedef struct
{
	uint8_t ctrlMode		: 2;	/*bit0  1=����ģʽ 0=�ֶ�ģʽ   bit1  1=����ģʽ*/
	uint8_t keyFlight 	: 1;	/*bit2 һ�����*/
	uint8_t keyLand 		: 1;	/*bit3 һ������*/
	uint8_t emerStop 	: 1;	/*bit4 ����ͣ��*/
	uint8_t flightMode 	: 1;	/*bit5 ����ģʽ 1=��ͷ 0=��ͷ*/
	uint8_t reserved		: 2;	/*bit6~7 ����*/
}commanderBits_t;

/*�������ݽṹ��*/
struct ctrlVal_t
{
	float roll;       // deg
	float pitch;      // deg
	float yaw;        // deg
	float trimPitch;
	float trimRoll;
	uint16_t thrust;
};

/*���ݻ���ṹ��*/
struct ctrlValCache_t
{
	struct ctrlVal_t  tarVal;
	uint32_t timestamp; 		/* FreeRTOS ʱ�ӽ���*/
};

typedef enum
{
	RATE    = 0,
	ANGLE   = 1,
} RPYType;

typedef enum
{
	XMODE     = 0, /*Xģʽ*/
	CAREFREE  = 1, /*��ͷģʽ*/
} YawModeType;

typedef enum
{
	ATK_REMOTER = 0
}ctrlSrc_e;

struct MiniFlyMsg_t
{
	uint8_t version;
	bool mpu_selfTest;
	bool baro_slfTest;
	bool isCanFly;
	bool isLowpower;
	
	float trimRoll;		/*roll΢��*/
	float trimPitch;	/*pitch΢��*/
};


typedef enum 
{
	REMOTER_CMD,
	REMOTER_DATA,
}remoterType_e;

typedef struct 
{
	uint32_t m1;
	uint32_t m2;
	uint32_t m3;
	uint32_t m4;
	
}motorPWM_t;

typedef struct
{
	float vAccDeadband; /* ���ٶ����� */
	float accBias[3];	/* ���ٶ� ƫ��(cm/s/s)*/
	float acc[3];		/* ������ٶ� ��λ(cm/s/s)*/
	float vel[3];		/* �����ٶ� ��λ(cm/s)*/
	float pos[3]; 		/* ����λ�� ��λ(cm)*/
} estimator_t;

typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	float desired;		//< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	float dt;           //< delta-time dt
	float out;			//< out
} PidObject;


struct remoteData_t
{
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[30];
};

typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;

typedef struct
{
	float pressure;
	float temperature;
	float asl;
} baro_t;

typedef enum 
{
	DOWN_COMMAND	= 0x01,
	DOWN_ACK		= 0x02,
	DOWN_RCDATA		= 0x03,
	DOWN_POWER		= 0x05,
	DOWN_FLYMODE	= 0x0A,
	DOWN_PID1		= 0x10,
	DOWN_PID2		= 0x11,
	DOWN_PID3		= 0x12,
	DOWN_PID4		= 0x13,
	DOWN_PID5		= 0x14,
	DOWN_PID6		= 0x15,
	DOWN_RADIO		= 0x40,
	DOWN_REMOTER	= 0x50,
}downmsgID_e;

struct remoterData_t
{
	float roll;      
	float pitch;  
	float yaw;      
	float thrust;
	float trimPitch;
	float trimRoll;
	uint8_t	ctrlMode;
	bool flightMode;
	bool RCLock;
} ;

#define ATKP_MAX_DATA_SIZE 30

typedef struct
{
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[ATKP_MAX_DATA_SIZE];
}atkp_t;

struct sensorData_t
{
	Axis3f acc;
	Axis3f gyro;
	baro_t baro;
};

//axis.h
typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

typedef struct
{
	pidInit_t roll;
	pidInit_t pitch;	
	pidInit_t yaw;	
} pidParam_t;

typedef struct
{
	pidInit_t vx;
	pidInit_t vy;
	pidInit_t vz;
	
	pidInit_t x;
	pidInit_t y;
	pidInit_t z;
} pidParamPos_t;

typedef struct	
{
	pidParam_t pidAngle;	/*??PID*/	
	pidParam_t pidRate;		/*???PID*/	
	pidParamPos_t pidPos;	/*??PID*/
	float trimP;			/*pitch??*/
	float trimR;			/*roll??*/
	uint16_t thrustBase;			/*?????*/
} configParam_t;

extern configParam_t configParam;

configParam_t configParam=
{
	.pidAngle=	/*??PID*/
	{	
		.roll=
		{
			.kp=8.0,
			.ki=0.0,
			.kd=0.0,
		},
		.pitch=
		{
			.kp=8.0,
			.ki=0.0,
			.kd=0.0,
		},
		.yaw=
		{
			.kp=20.0,
			.ki=0.0,
			.kd=1.5,
		},
	},	
	.pidRate=	/*???PID*/
	{	
		.roll=
		{
			.kp=300.0,
			.ki=0.0,
			.kd=6.5,
		},
		.pitch=
		{
			.kp=300.0,
			.ki=0.0,
			.kd=6.5,
		},
		.yaw=
		{
			.kp=200.0,
			.ki=18.5,
			.kd=0.0,
		},
	},	
	.pidPos=	/*??PID*/
	{	
		.vx=
		{
			.kp=4.5,
			.ki=0.0,
			.kd=0.0,
		},
		.vy=
		{
			.kp=4.5,
			.ki=0.0,
			.kd=0.0,
		},
		.vz=
		{
			.kp=100.0,
			.ki=150.0,
			.kd=10.0,
		},
		
		.x=
		{
			.kp=4.0,
			.ki=0.0,
			.kd=0.6,
		},
		.y=
		{
			.kp=4.0,
			.ki=0.0,
			.kd=0.6,
		},
		.z=
		{
			.kp=6.0,
			.ki=0.0,
			.kd=4.5,
		},
	},
	
	.trimP = 0.f,	/*pitch??*/
	.trimR = 0.f,	/*roll??*/
	.thrustBase=34000,	/*???????*/
};

//math.h
#ifndef sq
#define sq(x) ((x)*(x))
#endif

#define ABS(x) 		(((x) < 0) ? (-x) : (x))



/*pid�ṹ���ʼ��*/
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);/*pid�����޷�����*/
void pidSetOutputLimit(PidObject* pid, const float limit);
float pidUpdate(PidObject* pid, const float error);			/*pid����*/
void pidReset(PidObject* pid);			/*pid�ṹ�帴λ*/
void stateControlInit(void);
void stateControl(control_t *control, struct sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const uint32_t tick);
void attitudeControlInit(float rateDt, float angleDt);
void attitudeDataprocess(struct remoteData_t* anlPacket);
void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output);
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);
void attitudeControllerResetRollAttitudePID(void);
void attitudeControllerResetPitchAttitudePID(void);
void attitudeResetAllPID(void);
void positionControlInit(float ratePidDt, float posPidDt);
void positionDataprocess(struct remoteData_t* anlPacket);
void positionResetAllPID(void);
void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, float dt);
float getAltholdThrust(void);
void powerControlInit(void);
void powerDataprocess(struct remoteData_t* anlPacket);
bool powerControlTest(void);
void powerControl(control_t *control);
void setMotorPWM(bool enable, uint32_t m1_set, uint32_t m2_set, uint32_t m3_set, uint32_t m4_set);
void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt);	/*�����ں� �����˲�*/
bool getIsCalibrated(void);
void imuTransformVectorBodyToEarth(Axis3f * v);	/*���嵽����*/
void imuTransformVectorEarthToBody(Axis3f * v);	/*���򵽻���*/
void positionEstimate(struct sensorData_t* sensorData, state_t* state, float dt);	
float getFusedHeight(void);	/*��ȡ�ںϸ߶�*/
void estRstHeight(void);	/*��λ����߶�*/
void estRstAll(void);		/*��λ���й���*/
void anomalDetec(const struct sensorData_t *sensorData, const state_t *state, const control_t *control);
void remoterCtrlProcess(struct remoteData_t* pk);
void stabilizerInit(void);
void stabilizerTask(void* param);
bool stabilizerTest(void);
void getAttitudeData(attitude_t* get);
float getBaroData(void);
void getSensorData(struct sensorData_t* get);	
void getStateData(Axis3f* acc, Axis3f* vel, Axis3f* pos);
void setFastAdjustPosParam(uint16_t velTimes, uint16_t absTimes, float height);/*���ÿ��ٵ���λ�ò���*/
void flightCtrldataCache(ctrlSrc_e ctrlSrc, struct ctrlVal_t pk);
void commanderGetSetpoint(setpoint_t *setpoint, state_t *state);
void flyerAutoLand(setpoint_t *setpoint,const state_t *state);
void setCommanderCtrlMode(uint8_t set);
uint8_t getCommanderCtrlMode(void);
void setCommanderKeyFlight(bool set);
bool getCommanderKeyFlight(void);
void setCommanderKeyland(bool set);
bool getCommanderKeyland(void);
void setCommanderFlightmode(bool set);
void motorsInit(void);		/*�����ʼ��*/
bool motorsTest(void);		/*�������*/
void motorsSetRatio(uint32_t id, uint16_t ithrust);	/*���õ��ռ�ձ�*/
float applyDeadbandf(float value, float deadband);
float constrainf(float amt, float low, float high);
#endif /* __STABALIZER_H */
