/******************************************************************************
Filename    : stabilizer.c
Author      : fxy
Date        : 28/05/2025
Licence     : The Unlicense; see LICENSE for details.
Description : XXXXX
							XXXXX
******************************************************************************/

/* Include *******************************************************************/
#include"stm32f4xx.h"
#include "rvm.h"
#include "rmp.h"
#include <math.h>
#include "stabilizer.h"
/* End Include ***************************************************************/


/* 虚拟机通信相关 *************************************************************/

/* 共享内存指针 */
#define SHARED_SENSOR     ((volatile struct sensorData_t*)DATA_SHARED_SENSOR_BASE)
#define SHARED_REMOTE  		((volatile struct remoteData_t*)DATA_SHARED_REMOTE_BASE)

/* Sensor通信数据 */
struct Message_Sensor
{
    struct RMP_List Head;
	  volatile struct sensorData_t data;
};

/* Remote通信数据 */
struct Message_Remote
{
    struct RMP_List Head;
	  volatile struct remoteData_t data;
};

/* 内存池 */
struct RMP_List Pool_Sensor;
struct Message_Sensor msgarray_sensor[3];
struct RMP_List Pool_Remote;
struct Message_Remote msgarray_remote[3]; 

/* 消息队列 */
volatile struct RMP_Msgq Queue_Sensor;
volatile struct RMP_Msgq Queue_Remote;

/* Function:Sensor_Data_Read **************************************************
Description : Receive data from Sensor.
Input       : Pointer to Sensor_data.
Output      : None.
Return      : None.
******************************************************************************/
void Sensor_Data_Read(volatile struct sensorData_t* sensordata)
{
	volatile struct Message_Sensor* msg_S; 
	if(RMP_Msgq_Rcv(&Queue_Sensor,(volatile struct RMP_List**)(&msg_S),0)!=0)
		return;
	*sensordata=msg_S->data;
	RMP_INT_MASK();
	RMP_List_Ins((struct RMP_List*)(&msg_S),Pool_Sensor.Prev,Pool_Sensor.Next);
	RMP_INT_UNMASK();
}

/* Function:Remote_Data_Read **************************************************
Description : Receive data from Remote.
Input       : Pointer to data.
Output      : None.
Return      : None.
******************************************************************************/
bool Remote_Data_Read(struct remoteData_t *p)
{
  volatile struct Message_Remote* msg_R;
	if(RMP_Msgq_Rcv(&Queue_Remote,(volatile struct RMP_List**)(&msg_R),0)!=0)
		return false;
	*p = msg_R->data;
	RMP_INT_MASK();
	RMP_List_Ins((struct RMP_List*)(&msg_R),Pool_Remote.Prev,Pool_Remote.Next);
	RMP_INT_UNMASK();
	return true;
}

/* Function:Contact_Sensor ****************************************************
Description : Interrupt handling program with Sensor.
Input       : None.
Output      : None.
Return      : Whether the data was successfully read.
******************************************************************************/
void Contact_Sensor(void)
{
	volatile struct Message_Sensor* msg_s;
	if(Pool_Sensor.Next==&Pool_Sensor) return;
	msg_s=(volatile struct Message_Sensor*)(Pool_Sensor.Next);
	RMP_List_Del(msg_s->Head.Prev,msg_s->Head.Next);
	msg_s->data=*(struct sensorData_t*)SHARED_SENSOR;
	RMP_Msgq_Snd_ISR(&Queue_Sensor,(volatile struct RMP_List*)msg_s);
}

/* Function:Contact_Communicate ***********************************************
Description : Interrupt handling program with Communicate.
Input       : None.
Output      : None.
Return      : None.
******************************************************************************/
void Contact_Remote(void)
{
	volatile struct Message_Remote* msg_r;
	if(Pool_Remote.Next==&Pool_Remote) return;	
	msg_r=(volatile struct Message_Remote*)(Pool_Remote.Next);
	RMP_List_Del(msg_r->Head.Prev,msg_r->Head.Next);
	msg_r->data=*SHARED_REMOTE;
	RMP_Msgq_Snd_ISR(&Queue_Remote,(volatile struct RMP_List*)msg_r);
}

/* Function:Int_Init **********************************************************
Description : The init thread hook functions.
Input       : None.
Output      : None.
Return      : None.
******************************************************************************/

void Int_Init(void)
{
  /* Interrupt generation is initialized too, here we only register our handler */
	RVM_Virt_Vct_Reg(11U,Contact_Sensor);
  RVM_Virt_Vct_Reg(22U,Contact_Remote);
    
  /* Connect the virtual interrupt to our machine */
  RVM_Hyp_Vct_Evt(11U,11U);
  RVM_Hyp_Vct_Evt(22U,22U);

	/* Initialize memory pool */
	RMP_List_Crt(&Pool_Sensor);
	for(int i=0;i<3;i++){
		RMP_List_Ins(&msgarray_sensor[i].Head,Pool_Sensor.Prev,Pool_Sensor.Next);
	}
	RMP_List_Crt(&Pool_Remote);
	for(int i=0;i<3;i++){
		RMP_List_Ins(&msgarray_remote[i].Head,Pool_Remote.Prev,Pool_Remote.Next);
	}

	/* Initialize message queue */
	RMP_Msgq_Crt(&Queue_Sensor);
	RMP_Msgq_Crt(&Queue_Remote);

	/* Lock */
  RVM_Hyp_Vct_Lck();
}
/* End ***********************************************************************/


/* 电机相关 ******************************************************************/

/* PWM的分辨率为8位 */
#define MOTORS_PWM_BITS           	8

/* PWM的最大计数值 */
#define MOTORS_PWM_MAX         	255

/* 分频系数 */
#define MOTORS_PWM_PRESCALE       	0
									
/* 电机编号 */
#define MOTOR_M1  									0
#define MOTOR_M2  									1
#define MOTOR_M3  									2
#define MOTOR_M4  									3

/* 电机测试 */
#define MOTORS_TEST_PWM         		(uint16_t)(0.2*(1<<16))	
#define MOTORS_TEST_TIME_MS_ON    	50
#define MOTORS_TEST_TIME_MS_DELAY 	150

/* 保存四个电机的编号 */
static const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

/* 电机初始化检查 */
static bool motors_IsInit = false;

/* Function:Motors_Convert ****************************************************
Description : Convert duty cycle to CCR value to adapt to PWM outputs with different bit widths.
Output      : None.
Return      : CCR value.
******************************************************************************/
static uint16_t Motors_Convert(uint16_t val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

/* Motors_Init ****************************************************************
Description : Configure GPIO and timer to enable four-way motors to be controlled through PWM signals.
Input       : None.
Output      : None.
Return      : None.
******************************************************************************/
void Motors_Init(void)	
{
	/* 使能GPIO和定时器 */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;
	
	/* 复位定时器 */
	TIM4->CR1 = 0;
	TIM4->CCER = 0;
	TIM4->CCMR1 = 0;
	TIM4->CCMR2 = 0;
	TIM4->CNT = 0;
	TIM4->ARR = 0;
	TIM4->PSC = 0;
	
	TIM2->CR1 = 0;
	TIM2->CCER = 0;
	TIM2->CCMR1 = 0;
	TIM2->CCMR2 = 0;
	TIM2->CNT = 0;
	TIM2->ARR = 0;
	TIM2->PSC = 0;
	
	/* 配置PB7为TIM4 CH2 (MOTOR1) */
	GPIOB->MODER &= ~GPIO_MODER_MODER7;
	GPIOB->MODER |= GPIO_MODER_MODER7_1;  // ???????
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_7;   // ????
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7; // 100MHz
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR7;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;  // ??
	GPIOB->AFR[0] &= ~(0xF << 28);
	GPIOB->AFR[0] |= (2 << 28);           // AF2
	
	/* 配置PB6为TIM4 CH1 (MOTOR2) */
	GPIOB->MODER &= ~GPIO_MODER_MODER6;
	GPIOB->MODER |= GPIO_MODER_MODER6_1;  // ???????
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_6;   // ????
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6; // 100MHz
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR6;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;  // ??
	GPIOB->AFR[0] &= ~(0xF << 24);
	GPIOB->AFR[0] |= (2 << 24);           // AF2
	
	/* 配置PB10为TIM2 CH3 (MOTOR3) */
	GPIOB->MODER &= ~GPIO_MODER_MODER10;
	GPIOB->MODER |= GPIO_MODER_MODER10_1; // ???????
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_10;  // ????
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10; // 100MHz
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR10;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0; // ??
	GPIOB->AFR[1] &= ~(0xF << 8);
	GPIOB->AFR[1] |= (1 << 8);            // AF1
	
	/* 配置PA5为TIM2 CH1 (MOTOR4) */
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER5_1;  // ???????
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;   // ????
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5; // 100MHz
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_0;  // ??
	GPIOA->AFR[0] &= ~(0xF << 20);
	GPIOA->AFR[0] |= (1 << 20);           // AF1
	
	// ?????????
	TIM4->PSC = MOTORS_PWM_PRESCALE;     // ????
	TIM4->ARR = MOTORS_PWM_MAX;       // ??????
	TIM2->PSC = MOTORS_PWM_PRESCALE;     // ????
	TIM2->ARR = MOTORS_PWM_MAX;       // ??????
	
	// ??TIM4 CH1(PB6, MOTOR2)
	TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM??1
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;      // ?????
	TIM4->CCER |= TIM_CCER_CC1E;         // ????
	TIM4->CCR1 = 0;                      // ??????0
	
	// ??TIM4 CH2(PB7, MOTOR1)
	TIM4->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // PWM??1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;      // ?????
	TIM4->CCER |= TIM_CCER_CC2E;         // ????
	TIM4->CCR2 = 0;                      // ??????0
	
	// ??TIM2 CH3(PB10, MOTOR3)
	TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2); // PWM??1
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;      // ?????
	TIM2->CCER |= TIM_CCER_CC3E;         // ????
	TIM2->CCR3 = 0;                      // ??????0
	
	// ??TIM2 CH1(PA5, MOTOR4)
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM??1
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;      // ?????
	TIM2->CCER |= TIM_CCER_CC1E;         // ????
	TIM2->CCR1 = 0;                      // ??????0
	
	// ??ARR???
	TIM4->CR1 |= TIM_CR1_ARPE;
	TIM2->CR1 |= TIM_CR1_ARPE;
	
	// ?????
	TIM4->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 |= TIM_CR1_CEN;

	motors_IsInit = true;
}

/*????*/
bool motorsTest(void)
{
	int i;
	
	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{	
		motorsSetRatio(MOTORS[i], MOTORS_TEST_PWM);
		//*************delay_xms(MOTORS_TEST_TIME_MS_ON);
		motorsSetRatio(MOTORS[i], 0);
		//****************delay_xms(MOTORS_TEST_TIME_MS_DELAY);
	}

	return motors_IsInit;
}

/*????PWM???*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (motors_IsInit) 
	{
		u16 ratio=ithrust;
	
		float thrust = ((float)ithrust / 65536.0f) * 60;
		float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
		float supply_voltage = 3.7f;
		float percentage = volts / supply_voltage;
		percentage = percentage > 1.0f ? 1.0f : percentage;
		ratio = percentage * UINT16_MAX;
		
		u16 ccr_value = Motors_Convert(ratio);
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM4->CCR2 = ccr_value;
				break;
			case 1:		/*MOTOR_M2*/
				TIM4->CCR1 = ccr_value;
				break;
			case 2:		/*MOTOR_M3*/
				TIM2->CCR3 = ccr_value;
				break;
			case 3:		/*MOTOR_M4*/	
				TIM2->CCR1 = ccr_value;
				break;
			default: break;
		}	
	}
}




static bool isInit;


#define DEG2RAD		0.017453293f	
#define RAD2DEG		57.29578f

static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

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

#define THRUST_BASE  		(20000)	/*基础油门值*/

#define PIDVX_OUTPUT_LIMIT	120.0f	//ROLL限幅	(单位°带0.15的系数)
#define PIDVY_OUTPUT_LIMIT	120.0f 	//PITCH限幅	(单位°带0.15的系数)
#define PIDVZ_OUTPUT_LIMIT	(40000)	/*PID VZ限幅*/

#define PIDX_OUTPUT_LIMIT	1200.0f	//X轴速度限幅(单位cm/s 带0.1的系数)
#define PIDY_OUTPUT_LIMIT	1200.0f	//Y轴速度限幅(单位cm/s 带0.1的系数)
#define PIDZ_OUTPUT_LIMIT	120.0f	//Z轴速度限幅(单位cm/s)

static float thrustLpf = THRUST_BASE;	/*油门低通*/

PidObject pidVX;
PidObject pidVY;
PidObject pidVZ;

PidObject pidX;
PidObject pidY;
PidObject pidZ;

#define CLIMB_RATE			100.f
#define MAX_CLIMB_UP		100.f
#define MAX_CLIMB_DOWN		60.f

#define MIN_THRUST  	5000
#define MAX_THRUST  	60000

static bool isRCLocked;				/* 遥控锁定状态 */
static struct ctrlValCache_t remoteCache;	/* 遥控缓存数据 */
static struct ctrlVal_t ctrlValLpf = {0.f};/* 控制数据低通 */

static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 

static YawModeType yawMode = XMODE;	/* 默认为X飞行模式 */
static commanderBits_t commander;

static bool motorSetEnable = false;
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet={0, 0, 0, 0};

#define ACCZ_SAMPLE		350

float Kp = 0.4f;		/*比例增益*/
float Ki = 0.001f;		/*积分增益*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*积分误差累计*/

static float q0 = 1.0f;	/*四元数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float rMat[3][3];/*旋转矩阵*/

static float maxError = 0.f;		/*最大误差*/
bool isGravityCalibrated = false;	/*是否校校准完成*/
static float baseAcc[3] = {0.f,0.f,1.0f};	/*静态加速度*/

#define ACC_LIMIT			(1000.f)/*加速度限幅 单位cm/s/s*/
#define ACC_LIMIT_MAX		(1800.f)/*最大加速度限幅 单位cm/s/s*/
#define VELOCITY_LIMIT		(130.f)	/*速度限幅 单位cm/s*/
#define VELOCITY_LIMIT_MAX	(500.f)	/*最大速度限幅 单位cm/s*/

#define GRAVITY_CMSS 		(980.f)	/*重力加速度 单位cm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE	(GRAVITY_CMSS * 0.25f)   // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway


static float wBaro = 0.35f;			/*气压校正权重*/
static float wAccBias = 0.01f;		/*加速度校正权重*/

static bool isRstHeight = false;	/*复位高度*/
static bool isRstAll = true;		/*复位估测*/

static float fusedHeight;			/*融合高度，起飞点为0*/
static float fusedHeightLpf = 0.f;	/*融合高度，低通*/
static float startBaroAsl = 0.f;	/*起飞点海拔*/

/*估测系统*/
static estimator_t estimator = 
{
	.vAccDeadband = 4.0f,
	.accBias[0] =  0.0f,
	.accBias[1] =  0.0f,
	.accBias[2] =  0.0f,
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

static setpoint_t 	setpoint;	/*设置目标状态*/
struct sensorData_t sensorData;	/*传感器数据*/
struct remoteData_t pk;      /*从通信模块传过来的数据*/
static state_t 		state;		/*四轴姿态*/
static control_t 	control;	/*四轴控制参数*/

static uint16_t velModeTimes = 0;		/*速率模式次数*/
static uint16_t absModeTimes = 0;		/*绝对值模式次数*/
static float setHeight = 0.f;		/*设定目标高度 单位cm*/
static float baroLast = 0.f;
static float baroVelLpf = 0.f;

struct ctrlVal_t remoterCtrl;/*发送到commander姿态控制数据*/

void stabilizerTask(void* param);

void stabilizerInit(void)
{
	if(isInit) return;

	stateControlInit();		/*姿态PID初始化*/
	powerControlInit();		/*电机初始化*/

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;
	pass &= powerControlTest();
	return pass;
}


/*设置快速调整参数*/	
void setFastAdjustPosParam(uint16_t velTimes, uint16_t absTimes, float height)
{
	if(velTimes != 0 && velModeTimes == 0)
	{
		baroLast = sensorData.baro.asl;
		baroVelLpf = 0.f;

		velModeTimes = velTimes;
	}
	if(absTimes != 0 && absModeTimes ==0)
	{
		setHeight = height;
		absModeTimes = absTimes;
	}		
}

/*快速调整高度*/
static void fastAdjustPosZ(void)
{	
	if(velModeTimes > 0)
	{
		velModeTimes--;
		estRstHeight();	/*复位估测高度*/
		
		float baroVel = (sensorData.baro.asl - baroLast) / 0.004f;	/*250Hz*/
		baroLast = sensorData.baro.asl;
		baroVelLpf += (baroVel - baroVelLpf) * 0.35f;

		setpoint.mode.z = modeVelocity;
		state.velocity.z = baroVelLpf;		/*气压计融合*/
		setpoint.velocity.z = -1.0f * baroVelLpf;
		
	}
	else if(absModeTimes > 0)
	{
		absModeTimes--;
		estRstAll();	/*复位估测*/
		setpoint.mode.z = modeAbs;		
		setpoint.position.z = setHeight;
	}	
}

void stabilizerDataprocess(struct remoteData_t* anlPacket){
	if(anlPacket->msgID == DOWN_REMOTER)
	{
		remoterCtrlProcess(anlPacket);	/*遥控器数据处理*/
	}
	else if(anlPacket->msgID == DOWN_PID1 || anlPacket->msgID == DOWN_PID2)
	{
		attitudeDataprocess(anlPacket);	/*PID参数设置*/
	}	
	else if(anlPacket->msgID == DOWN_PID3 || anlPacket->msgID == DOWN_PID4)
	{
		positionDataprocess(anlPacket);	/*PID参数设置*/
	}
	else if(anlPacket->msgID == DOWN_PID6)
	{
		powerDataprocess(anlPacket);	/*电机参数设置*/
	}
}

void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	//???1ms
	
	while(1) 
	{

		//获取6轴和气压数据（500Hz）
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			Sensor_Data_Read(&sensorData);
						/*获取6轴和气压数据*/
		}

		// 获取通信模块发送的遥控数据,pid数据,电源数据,目标姿态和飞行模式设定（250Hz）	
		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			if(Remote_Data_Read(&pk))	/*接收数据*/
			{
				stabilizerDataprocess(&pk);	/*遥控数据,pid数据,电源数据处理*/
			}	
		}

		//四元数和欧拉角计算（250Hz）
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
		}

		//位置预估计算（250Hz）
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{
			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
		}
			
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && getIsCalibrated()==true)
		{
			commanderGetSetpoint(&setpoint, &state);	/*目标数据和飞行模式设定*/	
		}
		
		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			fastAdjustPosZ();	/*快速调整高度*/
		}		
		/*异常检测*/
		anomalDetec(&sensorData, &state, &control);			
		
		/*PID控制*/	
		
		stateControl(&control, &sensorData, &state, &setpoint, tick);
				
		
		//控制电机输出（500Hz）
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			powerControl(&control);	
		}
		
		tick++;
	}
}


void getAttitudeData(attitude_t* get)
{
	get->pitch = -state.attitude.pitch;
	get->roll = state.attitude.roll;
	get->yaw = -state.attitude.yaw;
}

float getBaroData(void)
{
	return sensorData.baro.asl;
}

void getSensorData(struct sensorData_t* get)
{
	*get = sensorData;
}

void getStateData(Axis3f* acc, Axis3f* vel, Axis3f* pos)
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

//remoter_ctrl.c

/*遥控数据接收处理*/
void remoterCtrlProcess(struct remoteData_t* pk)
{	
	if(pk->data[0] == REMOTER_CMD)
	{
		switch(pk->data[1])
		{
			case CMD_FLIGHT_LAND:
				if(getCommanderKeyFlight() != true)
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
	else if(pk->data[0] == REMOTER_DATA)
	{
		struct remoterData_t remoterData = *(struct remoterData_t*)(pk->data+1);
		
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

//anomal_detec.c

#if defined(DETEC_ENABLED)

static uint16_t outFlipCnt = 0;		


static bool detecFreeFall(float accZ, float accMAG)	/*自由落体检测*/
{
	static uint16_t cnt;

	/*自由落体*/
	if(fabs(accMAG) < DETEC_FF_THRESHOLD && fabs(accZ + 1.f) < DETEC_FF_THRESHOLD)	
	{	
		if(++cnt >= (DETEC_FF_COUNT))
		{
			return true;
		}		
	}
	else
	{
		cnt=0;
	}
	
	return false;
}

static bool detecTumbled(const state_t *state)	/*碰撞检测*/
{
	static uint16_t cnt;
	
	float fAbsRoll  = fabs(state->attitude.roll);
	float fAbsPitch = fabs(state->attitude.pitch);
	float fMax = (fAbsRoll >= fAbsPitch) ? fAbsRoll : fAbsPitch;
	
	if(fMax > DETEC_TU_THRESHOLD)
	{
		if(++cnt >= DETEC_TU_COUNT)
		{
			return true;
		}
	}else 
	{
		cnt=0;
	}
	
	return false;
}
#endif

/*异常检测*/
void anomalDetec(const struct sensorData_t *sensorData, const state_t *state, const control_t *control)
{
#if defined(DETEC_ENABLED)
	
	if(control->flipDir != CENTER) 
	{
		outFlipCnt = 1000;
		return;
	}	

	if(state->isRCLocked == false && 		//遥控器解锁状态
	getCommanderKeyFlight() == false &&		//未飞行状态
	(getCommanderCtrlMode() & 0x01) == 0x01)//定高模式
	{
		float accMAG = (sensorData->acc.x*sensorData->acc.x) +
						(sensorData->acc.y*sensorData->acc.y) +
						(sensorData->acc.z*sensorData->acc.z);

		if(detecFreeFall(state->acc.z/980.f, accMAG) == true)/*自由落体检测*/
		{				
			setCommanderKeyFlight(true);
			setFastAdjustPosParam(35, 10, 0.f);	/*设置快速调整位置参数*/
		}
	}
	
	if(outFlipCnt > 0)	
	{
		outFlipCnt--;
	}
	if(outFlipCnt == 0 && detecTumbled(state)==true)/*碰撞检测*/
	{
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}			

#endif
}

//state_estimator.c

/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
    estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt / 2.0f;
    estimator.vel[axis] += acc * dt;
}
/*位置校正*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    estimator.pos[axis] += ewdt;
    estimator.vel[axis] += w * ewdt;
}
/*速度校正*/
//static void inavFilterCorrectVel(int axis, float dt, float e, float w)
//{
//   estimator.vel[axis] += e * w * dt;
//}

void positionEstimate(struct sensorData_t* sensorData, state_t* state, float dt) 
{	
//	static float rangeLpf = 0.f;
	static float accLpf[3] = {0.f};		/*加速度低通*/	
	float weight = wBaro;

	float relateHight = sensorData->baro.asl - startBaroAsl;	/*气压相对高度*/
	
	fusedHeight = relateHight;	/*融合高度*/
	
	fusedHeightLpf += (fusedHeight - fusedHeightLpf) * 0.1f;	/*融合高度 低通*/
	
	if(isRstHeight)
	{	
		isRstHeight = false;
		
		weight = 0.95f;		/*增加权重，快速调整*/	
		
		startBaroAsl = sensorData->baro.asl;
		
		
		estimator.pos[Z] = fusedHeight;
	}
	else if(isRstAll)
	{
		isRstAll = false;
		
		accLpf[Z] = 0.f;	
		fusedHeight  = 0.f;
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
	
	estimator.acc[X] = applyDeadbandf(accelBF.x, estimator.vAccDeadband);/*去除死区的加速度*/
	estimator.acc[Y] = applyDeadbandf(accelBF.y, estimator.vAccDeadband);/*去除死区的加速度*/
	estimator.acc[Z] = applyDeadbandf(accelBF.z, estimator.vAccDeadband);/*去除死区的加速度*/
	
	for(uint8_t i=0; i<3; i++)
		accLpf[i] += (estimator.acc[i] - accLpf[i]) * 0.1f;	/*加速度低通*/
		
	bool isKeyFlightLand = ((getCommanderKeyFlight()==true)||(getCommanderKeyland()==true));	/*定高飞或者降落状态*/
	
	if(isKeyFlightLand == true)		/*定高飞或者降落状态*/
	{
		state->acc.x = constrainf(accLpf[X], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
		state->acc.y = constrainf(accLpf[Y], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
		state->acc.z = constrainf(accLpf[Z], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
	}else
	{
		state->acc.x = constrainf(estimator.acc[X], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
		state->acc.y = constrainf(estimator.acc[Y], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
		state->acc.z = constrainf(estimator.acc[Z], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
	}		

	
	float errPosZ = fusedHeight - estimator.pos[Z];
	
	/* 位置预估: Z-axis */
	inavFilterPredict(Z, dt, estimator.acc[Z]);
	/* 位置校正: Z-axis */
	inavFilterCorrectPos(Z, dt, errPosZ, weight);	

	
	/*加速度偏置校正*/
	Axis3f accelBiasCorr = {{ 0, 0, 0}};
	
	accelBiasCorr.z -= errPosZ  * sq(wBaro);
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

	if(isKeyFlightLand == true)		/*定高飞或者降落状态*/
	{
		state->velocity.x = constrainf(estimator.vel[X], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*速度限幅 VELOCITY_LIMIT*/
		state->velocity.y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*速度限幅 VELOCITY_LIMIT*/
		state->velocity.z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*速度限幅 VELOCITY_LIMIT*/
	}else
	{
		state->velocity.x = constrainf(estimator.vel[X], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*最大速度限幅 VELOCITY_LIMIT_MAX*/
		state->velocity.y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*最大速度限幅 VELOCITY_LIMIT_MAX*/
		state->velocity.z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*最大速度限幅 VELOCITY_LIMIT_MAX*/
	}
	
	state->position.x = estimator.pos[X];
	state->position.y = estimator.pos[Y];
	state->position.z = estimator.pos[Z];	
}//stabilizer.c

// /*读取融合高度 单位cm*/	
// float getFusedHeight(void)
// {
// 	return fusedHeightLpf;
// }

/*复位估测高度*/
void estRstHeight(void)
{
	isRstHeight = true;
}//stablilizer.c

/*复位所有估测*/
void estRstAll(void)
{
	isRstAll = true;
}//stabilizer.c + commander.c

//sensfusion6.c

static float invSqrt(float x);	/*快速开平方求倒*/

static void calBaseAcc(float* acc)	/*计算静态加速度*/
{
	static uint16_t cnt = 0;
	static float accZMin = 1.5;
	static float accZMax = 0.5;
	static float sumAcc[3] = {0.f};
	
	for(uint8_t i=0; i<3; i++)
		sumAcc[i] += acc[i];
		
	if(acc[2] < accZMin)	accZMin = acc[2];
	if(acc[2] > accZMax)	accZMax = acc[2];
	
	if(++cnt >= ACCZ_SAMPLE) /*缓冲区满*/
	{
		cnt = 0;
		maxError = accZMax - accZMin;
		accZMin = 1.5;
		accZMax = 0.5;
		
		if(maxError < 0.015f)
		{
			for(uint8_t i=0; i<3; i++)
				baseAcc[i] = sumAcc[i] / ACCZ_SAMPLE;
			
			isGravityCalibrated = true;
		}
		
		for(uint8_t i=0; i<3; i++)		
			sumAcc[i] = 0.f;		
	}	
}

/*计算旋转矩阵*/
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

void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt)	/*数据融合 互补滤波*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	Axis3f tempacc = acc;
	
	gyro.x = gyro.x * DEG2RAD;	/* 度转弧度 */
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;

	/* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
	if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		/*单位化加速计测量值*/
		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;

		/*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
		ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
		ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
		ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);
		
		/*误差累计，与积分常数相乘*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
		gyro.x += Kp * ex + exInt;
		gyro.y += Kp * ey + eyInt;
		gyro.z += Kp * ez + ezInt;
	}
	/* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
	q1 += ( q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
	q2 += ( q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
	q3 += ( q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;
	
	/*单位化四元数*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	/*计算旋转矩阵*/
	
	/*计算roll pitch yaw 欧拉角*/
	state->attitude.pitch = -asinf(rMat[2][0]) * RAD2DEG; 
	state->attitude.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	state->attitude.yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
	
	if (!isGravityCalibrated)	/*未校准*/
	{		
//		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
//		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
		accBuf[2] = tempacc.x* rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2];	/*accz*/
		calBaseAcc(accBuf);		/*计算静态加速度*/				
	}
}//stablilizer.c

/*机体到地球*/
void imuTransformVectorBodyToEarth(Axis3f * v)
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
    v->z = z - baseAcc[2] *  980.f;	/*去除重力加速度*/
}//state_estimator.c

/*地球到机体*/
void imuTransformVectorEarthToBody(Axis3f * v)
{
    v->y = -v->y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->x + rMat[1][0] * v->y + rMat[2][0] * v->z;
    const float y = rMat[0][1] * v->x + rMat[1][1] * v->y + rMat[2][1] * v->z;
    const float z = rMat[0][2] * v->x + rMat[1][2] * v->y + rMat[2][2] * v->z;

    v->x= x;
    v->y = y;
    v->z = z;
}//state_estimator.c

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)	/*快速开平方求倒*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

bool getIsCalibrated(void)
{
	return isGravityCalibrated;
}//stabilizer.c

//power_control.c

void powerControlInit(void)
{
	Motors_Init();
}//stabilizer.c

void powerDataprocess(struct remoteData_t* anlPacket)
{
	int16_t enable = ((int16_t)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
	int16_t m1_set = ((int16_t)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
	int16_t m2_set = ((int16_t)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
	int16_t m3_set = ((int16_t)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
	int16_t m4_set = ((int16_t)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
	setMotorPWM(enable,m1_set,m2_set,m3_set,m4_set);
}//stabilizer.c

bool powerControlTest(void)
{
	bool pass = true;

	pass &= motorsTest();

	return pass;
}//stabilizer.c

uint16_t limitThrust(int value)
{
	if(value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if(value < 0)
	{
		value = 0;
	}

	return (uint16_t)value;
}

void powerControl(control_t *control)	/*功率输出控制*/
{
	int16_t r = control->roll / 2.0f;
	int16_t p = control->pitch / 2.0f;
	
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
}//stabilizer.c

// void getMotorPWM(motorPWM_t* get)
// {
// 	*get = motorPWM;
// }

void setMotorPWM(bool enable, uint32_t m1_set, uint32_t m2_set, uint32_t m3_set, uint32_t m4_set)
{
	motorSetEnable = enable;
	motorPWMSet.m1 = m1_set;
	motorPWMSet.m2 = m2_set;
	motorPWMSet.m3 = m3_set;	
	motorPWMSet.m4 = m4_set;
}

//commander.c

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
	if(commander.keyFlight)	/*飞行过程中，遥控器信号断开，一键降落*/
	{
		commander.keyLand = true;
		commander.keyFlight = false;
	}	
}
uint32_t timestamp = 0;
/********************************************************
 *ctrlDataUpdate()	更新控制数据
 *遥控数据 优先级高于wifi控制数据
*********************************************************/
static void ctrlDataUpdate(void)	
{
	static float lpfVal = 0.2f;
	//uint32_t tickNow = getSysTickCnt();	

	
	//if ((tickNow - remoteCache.timestamp) < COMMANDER_WDT_TIMEOUT_STABILIZE) 
	//{
	//	isRCLocked = false;			/*解锁*/
	//}else 
	//if ((tickNow - remoteCache.timestamp) < COMMANDER_WDT_TIMEOUT_SHUTDOWN) 
	//{
	//	commanderLevelRPY();
	//}else 
	//{
	//	isRCLocked = true;			/*锁定*/
	//	commanderDropToGround();
	//}
	isRCLocked = false;
	
	if(isRCLocked == false)	/*解锁状态*/
	{
		struct ctrlVal_t ctrlVal =  remoteCache.tarVal;	/*读取缓存*/
		
		ctrlValLpf.thrust += (ctrlVal.thrust - ctrlValLpf.thrust) * lpfVal;
		ctrlValLpf.pitch += (ctrlVal.pitch - ctrlValLpf.pitch) * lpfVal;
		ctrlValLpf.roll += (ctrlVal.roll - ctrlValLpf.roll) * lpfVal;
		ctrlValLpf.yaw += (ctrlVal.yaw - ctrlValLpf.yaw) * lpfVal;
		
		configParam.trimP = ctrlVal.trimPitch;	/*更新微调值*/
		configParam.trimR = ctrlVal.trimRoll;
		
		if (ctrlValLpf.thrust < MIN_THRUST)
			ctrlValLpf.thrust = 0;	
		else 		
			ctrlValLpf.thrust = (ctrlValLpf.thrust>=MAX_THRUST) ? MAX_THRUST:ctrlValLpf.thrust;
	}
}

/************************************************************************
* 四轴carefree(无头模式)，参考世界坐标系，当四轴围绕YAW旋转后，
* 四轴前方任然保持开始的方向，这个模式对新手非常实用
************************************************************************/
static void rotateYawCarefree(setpoint_t *setpoint, const state_t *state)
{
	float yawRad = state->attitude.yaw * DEG2RAD;
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	
	if(setpoint->mode.x ==  modeDisable || setpoint->mode.y ==  modeDisable)	/*手动和定高模式*/
	{
		float originalRoll = setpoint->attitude.roll;
		float originalPitch = setpoint->attitude.pitch;

		setpoint->attitude.roll = originalRoll * cosy + originalPitch * siny;
		setpoint->attitude.pitch = originalPitch * cosy - originalRoll * siny;
	}
	else if(setpoint->mode.x ==  modeVelocity || setpoint->mode.y ==  modeVelocity)	/*定点模式*/
	{
		float originalVy = setpoint->velocity.y;
		float originalVx = setpoint->velocity.x;

		setpoint->velocity.y = originalVy * cosy + originalVx * siny;
		setpoint->velocity.x = originalVx * cosy - originalVy * siny;
	}
}

/*飞控数据缓存*/
void flightCtrldataCache(ctrlSrc_e ctrlSrc, struct ctrlVal_t pk)
{
	remoteCache.tarVal = pk;
	//remoteCache.timestamp = getSysTickCnt();
}//remoter_ctrl
//stavilier.c


/********************************************************
* flyerAutoLand()
* 四轴自动降落
*********************************************************/
void flyerAutoLand(setpoint_t *setpoint,const state_t *state)
{	
	static uint8_t lowThrustCnt = 0;
	static float stateVelLpf  = -30.f;
	
	setpoint->mode.z = modeVelocity;
	stateVelLpf += (state->velocity.z -  stateVelLpf) * 0.1f;	/*速率低通*/
	setpoint->velocity.z = -70.f - stateVelLpf;	/*降落速度 单位cm/s*/

	if(getAltholdThrust() < 20000.f)	/*定高油门值较低*/
	{
		lowThrustCnt++;
		if(lowThrustCnt > 10)
		{
			lowThrustCnt = 0;
			commander.keyLand = false;
			commander.keyFlight = false;
			estRstAll();	/*复位估测*/
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
		estRstAll();	/*复位估测*/
	}	
}

static bool initHigh = false;
static bool isAdjustingPosZ = false;/*调整Z位置*/
 static float errorPosZ = 0.f;		/*Z位移误差*/


void commanderGetSetpoint(setpoint_t *setpoint, state_t *state)
{	
	static float maxAccZ = 0.f;
	
	ctrlDataUpdate();	/*更新控制数据*/
	
	state->isRCLocked = isRCLocked;	/*更新遥控器锁定状态*/
	
	if(commander.ctrlMode & 0x01)/*定高模式*/
	{
		if(commander.keyLand)/*一键降落*/
		{
			flyerAutoLand(setpoint, state);
		}
		else if(commander.keyFlight)/*一键起飞*/ 
		{	
			setpoint->thrust = 0;
			setpoint->mode.z = modeAbs;		
			
			if (initHigh == false)
			{
				initHigh = true;	
				errorPosZ = 0.f;

				setFastAdjustPosParam(0, 1, 80.f);	/*一键起飞高度80cm*/															
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

				if(climb < -(CLIMB_RATE/5.f))	/*油门下拉过大*/
				{
					if(maxAccZ < state->acc.z)
						maxAccZ = state->acc.z;
					if(maxAccZ > 250.f)		/*油门下拉过大，飞机触地停机*/
					{
						commander.keyFlight = false;
						estRstAll();	/*复位估测*/
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
				setpoint->position.z = state->position.z + errorPosZ;	/*调整新位置*/									
			}
			else if(isAdjustingPosZ == false)	/*Z位移误差*/
			{
				errorPosZ = setpoint->position.z - state->position.z;
				errorPosZ = constrainf(errorPosZ, -10.f, 10.f);	/*误差限幅 单位cm*/
			}			
		}
		else/*着陆状态*/
		{
			setpoint->mode.z = modeDisable;
			setpoint->thrust = 0;
			setpoint->velocity.z = 0;
			setpoint->position.z = 0;
			initHigh = false;
			isAdjustingPosZ = false;
		}
	}
	else /*手动飞模式*/
	{
		setpoint->mode.z = modeDisable;
		setpoint->thrust = ctrlValLpf.thrust;
	}	
 	
	setpoint->attitude.roll = ctrlValLpf.roll;
	setpoint->attitude.pitch = ctrlValLpf.pitch;
	setpoint->attitude.yaw  = -ctrlValLpf.yaw;	/*摇杆方向和yaw方向相反*/
	
		setpoint->mode.x = modeDisable;
		setpoint->mode.y = modeDisable;		
	
	setpoint->mode.roll = modeDisable;	
	setpoint->mode.pitch = modeDisable;	
	
	if(commander.flightMode)/*无头模式*/
	{
		yawMode = CAREFREE;		
		rotateYawCarefree(setpoint, state);
	}		
	else	/*X飞行模式*/
	{
		yawMode = XMODE;
	}		
}//stabilizer.c

/* 读取并更新微调值 */
// void getAndUpdateTrim(float* pitch, float* roll)
// {
// 	*pitch = remoteCache.tarVal.trimPitch;
// 	*roll = remoteCache.tarVal.trimRoll;
// }

void setCommanderCtrlMode(uint8_t set)
{
	commander.ctrlMode = (set & 0x03);
}//remoter_ctrl.c
uint8_t getCommanderCtrlMode(void)
{
	return (commander.ctrlMode & 0x03);
}//anomal_detec.c

// uint8_t getCommanderFlightMode(void)
// {
// 	return (yawMode & 0x01);
// }

void setCommanderKeyFlight(bool set)
{
	commander.keyFlight = set;
	if(set == true)	/*一键起飞，清零最大最小值*/
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

//state_control.c

void stateControlInit(void)
{
	attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT); /*初始化姿态PID*/	
	positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*初始化位置PID*/
}

void stateControl(control_t *control, struct sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const uint32_t tick)
{
	static uint16_t cnt = 0;
	
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

void attitudeDataprocess(struct remoteData_t* anlPacket){
	if(anlPacket->msgID == DOWN_PID1)
	{
		pidRateRoll.kp  = 0.1*((int16_t)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidRateRoll.ki  = 0.1*((int16_t)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidRateRoll.kd  = 0.1*((int16_t)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidRatePitch.kp = 0.1*((int16_t)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidRatePitch.ki = 0.1*((int16_t)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidRatePitch.kd = 0.1*((int16_t)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidRateYaw.kp   = 0.1*((int16_t)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidRateYaw.ki   = 0.1*((int16_t)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidRateYaw.kd   = 0.1*((int16_t)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
	}
	else if(anlPacket->msgID == DOWN_PID2)
	{
		pidAngleRoll.kp  = 0.1*((int16_t)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidAngleRoll.ki  = 0.1*((int16_t)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidAngleRoll.kd  = 0.1*((int16_t)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidAnglePitch.kp = 0.1*((int16_t)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidAnglePitch.ki = 0.1*((int16_t)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidAnglePitch.kd = 0.1*((int16_t)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidAngleYaw.kp   = 0.1*((int16_t)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidAngleYaw.ki   = 0.1*((int16_t)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidAngleYaw.kd   = 0.1*((int16_t)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
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


void positionControlInit(float velocityPidDt, float posPidDt)
{
	pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt);	/*vx PID初始化*/
	pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt);	/*vy PID初始化*/
	pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt);	/*vz PID初始化*/
	pidSetOutputLimit(&pidVX, PIDVX_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidVY, PIDVY_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidVZ, PIDVZ_OUTPUT_LIMIT);		/* 输出限幅 */
	
	pidInit(&pidX, 0, configParam.pidPos.x, posPidDt);			/*x PID初始化*/
	pidInit(&pidY, 0, configParam.pidPos.y, posPidDt);			/*y PID初始化*/
	pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt);			/*z PID初始化*/
	pidSetOutputLimit(&pidX, PIDX_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidY, PIDY_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidZ, PIDZ_OUTPUT_LIMIT);		/* 输出限幅 */
}//state_control

void positionDataprocess(struct remoteData_t* anlPacket){
	if(anlPacket->msgID == DOWN_PID3)
	{
		pidVZ.kp = 0.1*((int16_t)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidVZ.ki = 0.1*((int16_t)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidVZ.kd = 0.1*((int16_t)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		pidZ.kp = 0.1*((int16_t)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidZ.ki = 0.1*((int16_t)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidZ.kd = 0.1*((int16_t)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		pidVX.kp = 0.1*((int16_t)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidVX.ki = 0.1*((int16_t)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidVX.kd = 0.1*((int16_t)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		
		pidVY = pidVX;	//位置速率PID，X\Y方向是一样的
	}
	else if(anlPacket->msgID == DOWN_PID4)
	{
		pidX.kp = 0.1*((int16_t)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidX.ki = 0.1*((int16_t)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidX.kd = 0.1*((int16_t)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		pidY = pidX;	//位置保持PID，X\Y方向是一样的
	}
}//stabilizer

static void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)                                                         
{	
	static uint16_t altholdCount = 0;
	
	// Roll and Pitch
	attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
	attitude->roll = 0.15f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);
	
	// Thrust
	float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);
	
	*thrust = constrainf(thrustRaw + THRUST_BASE, 1000, 60000);	/*油门限幅*/
	
	thrustLpf += (*thrust - thrustLpf) * 0.003f;
	
	if(getCommanderKeyFlight())	/*定高飞行状态*/
	{
		if(fabs(state->acc.z) < 35.f)
		{
			altholdCount++;
			if(altholdCount > 1000)
			{
				altholdCount = 0;
				if(fabs(configParam.thrustBase - thrustLpf) > 1000.f)	/*更新基础油门值*/
					configParam.thrustBase = thrustLpf;
			}
		}else
		{
			altholdCount = 0;
		}
	}else if(getCommanderKeyland() == false)	/*降落完成，油门清零*/
	{
		*thrust = 0;
	}
}//state_control

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
}//state_control

/*获取定高油门值*/
float getAltholdThrust(void)
{
	return thrustLpf;
}//commander.c

void positionResetAllPID(void)
{
	pidReset(&pidVX);
	pidReset(&pidVY);
	pidReset(&pidVZ);

	pidReset(&pidX);
	pidReset(&pidY);
	pidReset(&pidZ);
}


//pid.c

void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired = desired;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
	pid->outputLimit = DEFAULT_PID_OUTPUT_LIMIT;
	pid->dt = dt;
}//state_control

float pidUpdate(PidObject* pid, const float error)
{
	float output;

	pid->error = error;   

	pid->integ += pid->error * pid->dt;
	
	//积分限幅
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
	
	//输出限幅
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
}//state_control

void pidSetIntegralLimit(PidObject* pid, const float limit) 
{
    pid->iLimit = limit;
}//state_control

void pidSetOutputLimit(PidObject* pid, const float limit) 
{
	pid->outputLimit = limit;
}//state_control

void pidReset(PidObject* pid)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
}//state_control


float applyDeadbandf(float value, float deadband)
{
    if (ABS(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
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