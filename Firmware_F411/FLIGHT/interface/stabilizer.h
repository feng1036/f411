#ifndef __STABALIZER_H
#define __STABALIZER_H
#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"
#include "atkp.h"
#include "config_param.h"
#include "sys.h"

#define DEFAULT_PID_INTEGRATION_LIMIT 		500.0 //默认pid的积分限幅
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0	  //默认pid输出限幅，0为不限幅
#define DETEC_ENABLED
#define DETEC_FF_THRESHOLD 					0.05f	/* accZ接近-1.0程度 表示Free Fall */
#define DETEC_FF_COUNT 						50  	/* 自由落体检测计数 1000Hz测试条件 */
#define DETEC_TU_THRESHOLD 					60		/* 碰撞检测阈值60°*/
#define DETEC_TU_COUNT 						100  	/* 碰撞检测计数 1000Hz测试条件 */
#define CMD_GET_CANFLY						0x02	/*获取四轴是否能飞*/
#define CMD_FLIGHT_LAND						0x03	/*起飞、降落*/
#define CMD_EMER_STOP						0x04	/*紧急停机*/
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
#define MAIN_LOOP_DT			(u32)(1000/MAIN_LOOP_RATE)	/*单位ms*/

#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/RATE_250_HZ)

#define POSITION_ESTIMAT_RATE	RATE_250_HZ	//位置预估速率
#define POSITION_ESTIMAT_DT		(1.0/RATE_250_HZ)

#define RATE_PID_RATE			RATE_500_HZ //角速度环（内环）PID速率
#define RATE_PID_DT				(1.0/RATE_500_HZ)

#define ANGEL_PID_RATE			ATTITUDE_ESTIMAT_RATE //角度环（外环）PID速率
#define ANGEL_PID_DT			(1.0/ATTITUDE_ESTIMAT_RATE)

#define VELOCITY_PID_RATE		POSITION_ESTIMAT_RATE //速度环（内环）PID速率
#define VELOCITY_PID_DT			(1.0/POSITION_ESTIMAT_RATE)

#define POSITION_PID_RATE		POSITION_ESTIMAT_RATE //位置环（外环）PID速率
#define POSITION_PID_DT			(1.0/POSITION_ESTIMAT_RATE)

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000

typedef struct
{
	u8 ctrlMode		: 2;	/*bit0  1=定高模式 0=手动模式   bit1  1=定点模式*/
	u8 keyFlight 	: 1;	/*bit2 一键起飞*/
	u8 keyLand 		: 1;	/*bit3 一键降落*/
	u8 emerStop 	: 1;	/*bit4 紧急停机*/
	u8 flightMode 	: 1;	/*bit5 飞行模式 1=无头 0=有头*/
	u8 reserved		: 2;	/*bit6~7 保留*/
}commanderBits_t;

/*控制数据结构体*/
typedef __packed struct
{
	float roll;       // deg
	float pitch;      // deg
	float yaw;        // deg
	float trimPitch;
	float trimRoll;
	u16 thrust;
} ctrlVal_t;

/*数据缓存结构体*/
typedef struct
{
	ctrlVal_t  tarVal;
	u32 timestamp; 		/* FreeRTOS 时钟节拍*/
} ctrlValCache_t;

typedef enum
{
	RATE    = 0,
	ANGLE   = 1,
} RPYType;

typedef enum
{
	XMODE     = 0, /*X模式*/
	CAREFREE  = 1, /*无头模式*/
} YawModeType;

typedef enum
{
	ATK_REMOTER = 0
}ctrlSrc_e;

typedef __packed struct
{
	float roll;      
	float pitch;  
	float yaw;      
	float thrust;
	float trimPitch;
	float trimRoll;
	u8	ctrlMode;
	bool flightMode;
	bool RCLock;
} remoterData_t;

typedef __packed struct
{
	u8 version;
	bool mpu_selfTest;
	bool baro_slfTest;
	bool isCanFly;
	bool isLowpower;
	
	float trimRoll;		/*roll微调*/
	float trimPitch;	/*pitch微调*/
} MiniFlyMsg_t;


typedef enum 
{
	REMOTER_CMD,
	REMOTER_DATA,
}remoterType_e;

typedef struct 
{
	u32 m1;
	u32 m2;
	u32 m3;
	u32 m4;
	
}motorPWM_t;

typedef struct
{
	float vAccDeadband; /* 加速度死区 */
	float accBias[3];	/* 加速度 偏置(cm/s/s)*/
	float acc[3];		/* 估测加速度 单位(cm/s/s)*/
	float vel[3];		/* 估测速度 单位(cm/s)*/
	float pos[3]; 		/* 估测位移 单位(cm)*/
} estimator_t;

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


/*pid结构体初始化*/
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);/*pid积分限幅设置*/
void pidSetOutputLimit(PidObject* pid, const float limit);
float pidUpdate(PidObject* pid, const float error);			/*pid更新*/
void pidReset(PidObject* pid);			/*pid结构体复位*/
void stateControlInit(void);
void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const u32 tick);
void attitudeControlInit(float rateDt, float angleDt);
void attitudeDataprocess(atkp_t* anlPacket);
void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output);
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);
void attitudeControllerResetRollAttitudePID(void);
void attitudeControllerResetPitchAttitudePID(void);
void attitudeResetAllPID(void);
void positionControlInit(float ratePidDt, float posPidDt);
void positionDataprocess(atkp_t* anlPacket);
void positionResetAllPID(void);
void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, float dt);
float getAltholdThrust(void);
void powerControlInit(void);
void powerDataprocess(atkp_t* anlPacket);
bool powerControlTest(void);
void powerControl(control_t *control);
void setMotorPWM(bool enable, u32 m1_set, u32 m2_set, u32 m3_set, u32 m4_set);
void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt);	/*数据融合 互补滤波*/
bool getIsCalibrated(void);
void imuTransformVectorBodyToEarth(Axis3f * v);	/*机体到地球*/
void imuTransformVectorEarthToBody(Axis3f * v);	/*地球到机体*/
void positionEstimate(sensorData_t* sensorData, state_t* state, float dt);	
float getFusedHeight(void);	/*读取融合高度*/
void estRstHeight(void);	/*复位估测高度*/
void estRstAll(void);		/*复位所有估测*/
void anomalDetec(const sensorData_t *sensorData, const state_t *state, const control_t *control);
void remoterCtrlProcess(atkp_t* pk);
void stabilizerInit(void);
void stabilizerTask(void* param);
bool stabilizerTest(void);
void getAttitudeData(attitude_t* get);
float getBaroData(void);
void getSensorData(sensorData_t* get);	
void getStateData(Axis3f* acc, Axis3f* vel, Axis3f* pos);
void setFastAdjustPosParam(u16 velTimes, u16 absTimes, float height);/*设置快速调整位置参数*/
void flightCtrldataCache(ctrlSrc_e ctrlSrc, ctrlVal_t pk);
void commanderGetSetpoint(setpoint_t *setpoint, state_t *state);
void flyerAutoLand(setpoint_t *setpoint,const state_t *state);
void setCommanderCtrlMode(u8 set);
u8 getCommanderCtrlMode(void);
void setCommanderKeyFlight(bool set);
bool getCommanderKeyFlight(void);
void setCommanderKeyland(bool set);
bool getCommanderKeyland(void);
void setCommanderFlightmode(bool set);
#endif /* __STABALIZER_H */
