#include <math.h>
#include <stdio.h>
#include "sensors.h"

static bool gyroBiasFound=false;
static bool isBaroPresent=false;
static float accScaleSum=0;
static float accScale=1;
static enum {IDLE, BMP280, SPL06}baroType = IDLE;
static bool isInit=false;
/*传感器中断初始化*/
static void sensorsInterruptInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/*使能MPU6500中断*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);

	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	portDISABLE_INTERRUPTS();
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line4);
	portENABLE_INTERRUPTS();
}

/* 传感器器件初始化 */
void sensorsDeviceInit(void)
{
	i2cdevInit(I2C1_DEV);
	mpu6500Init(I2C1_DEV);
	vTaskDelay(10);
	mpu6500Reset();	// 复位MPU6500
	vTaskDelay(20);	// 延时等待寄存器复位	
	mpu6500SetSleepEnabled(false);	// 唤醒MPU6500
	vTaskDelay(10);		
	mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);	// 设置X轴陀螺作为时钟	
	vTaskDelay(10);		// 延时等待时钟稳定	
	mpu6500SetTempSensorEnabled(true);	// 使能温度传感器	
	mpu6500SetIntEnabled(false);		// 关闭中断	
	mpu6500SetI2CBypassEnabled(true);	// 旁路模式，磁力计和气压连接到主IIC	
	mpu6500SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);	// 设置陀螺量程	
	mpu6500SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);// 设置加速计量程	
	mpu6500SetAccelDLPF(MPU6500_ACCEL_DLPF_BW_41);		// 设置加速计数字低通滤波

	mpu6500SetRate(0);// 设置采样速率: 1000 / (1 + 0) = 1000Hz
	mpu6500SetDLPFMode(MPU6500_DLPF_BW_98);// 设置陀螺数字低通滤波
	
	for (u8 i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}


	if (bmp280Init(I2C1_DEV) == true)//BMP280初始化
	{
		isBaroPresent = true;
		baroType = BMP280;
		vTaskDelay(100);
	}
	else
	{
		isBaroPresent = false;
	}
}
/*传感器偏置初始化*/
static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}

/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
	u32 i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*传感器查找偏置值*/
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		
		Axis3f mean;
		Axis3f variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}

/* 传感器初始化 */
void sensorsInit(void)
{
	if(isInit) return;
	queue_init(1);
	sensorsDataReady = xSemaphoreCreateBinary();/*创建传感器数据就绪二值信号量*/
	sensorsBiasObjInit(&gyroBiasRunning);
	sensorsDeviceInit();	/*传感器器件初始化*/
	sensorsInterruptInit();	/*传感器中断初始化*/
	isInit = true;
}
/*设置传感器从模式读取*/
static void sensorsSetupSlaveRead(void)
{
	mpu6500SetSlave4MasterDelay(19); 	// 从机读取速率: 100Hz = (1000Hz / (1 + 9))

	mpu6500SetI2CBypassEnabled(false);	//主机模式
	mpu6500SetWaitForExternalSensorEnabled(true); 	
	mpu6500SetInterruptMode(0); 		// 中断高电平有效
	mpu6500SetInterruptDrive(0); 		// 推挽输出
	mpu6500SetInterruptLatch(0); 		// 中断锁存模式(0=50us-pulse, 1=latch-until-int-cleared)
	mpu6500SetInterruptLatchClear(1); 	// 中断清除模式(0=status-read-only, 1=any-register-read)
	mpu6500SetSlaveReadWriteTransitionEnabled(false); // 关闭从机读写传输
	mpu6500SetMasterClockSpeed(13); 	// 设置i2c速度400kHz

	if (isBaroPresent && baroType == BMP280)
	{
		// 设置MPU6500主机要读取BMP280的寄存器
		mpu6500SetSlaveAddress(1, 0x80 | BMP280_I2C_ADDR);		// 设置气压计状态寄存器为1号从机
		mpu6500SetSlaveRegister(1, BMP280_STAT_REG);			// 从机1需要读取的寄存器
		mpu6500SetSlaveDataLength(1, SENSORS_BARO_STATUS_LEN);	// 读取1个字节
		mpu6500SetSlaveDelayEnabled(1, true);
		mpu6500SetSlaveEnabled(1, true);

		mpu6500SetSlaveAddress(2, 0x80 | BMP280_I2C_ADDR);		// 设置气压计数据寄存器为2号从机
		mpu6500SetSlaveRegister(2, BMP280_PRESSURE_MSB_REG);	// 从机2需要读取的寄存器
		mpu6500SetSlaveDataLength(2, SENSORS_BARO_DATA_LEN);	// 读取6个字节
		mpu6500SetSlaveDelayEnabled(2, true);
		mpu6500SetSlaveEnabled(2, true);
	}
	mpu6500SetI2CMasterModeEnabled(true);	//使能mpu6500主机模式
	mpu6500SetIntDataReadyEnabled(true);	//数据就绪中断使能
}

/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}

/**
 * 根据样本计算重力加速度缩放因子
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}
	return accBiasFound;
}

/**
 * 计算陀螺方差
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

/*处理气压计数据*/
void processBarometerMeasurements(const u8 *buffer)
{
	static float temp;
    static float pressure;
	
	if (baroType == BMP280)
	{
		// Check if there is a new data update
		if ((buffer[0] & 0x08)) /*bit3=1 转换完成*/
		{
			s32 rawPressure = (s32)((((u32)(buffer[1])) << 12) | (((u32)(buffer[2])) << 4) | ((u32)buffer[3] >> 4));
			s32 rawTemp = (s32)((((u32)(buffer[4])) << 12) | (((u32)(buffer[5])) << 4) | ((u32)buffer[6] >> 4));
			temp = bmp280CompensateT(rawTemp)/100.0f;		
			pressure = bmp280CompensateP(rawPressure)/25600.0f;			
			sensors.baro.pressure = pressure;
			sensors.baro.temperature = (float)temp;	/*单位度*/
			sensors.baro.asl = bmp280PressureToAltitude(&pressure) * 100.f;	/*转换成海拔*/
		}
	}
}
/*处理加速计和陀螺仪数据*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	/*注意传感器读取方向(旋转270°x和y交换)*/
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	/*计算accScale*/
	}
	
	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf(gyroLpf, &sensors.gyro);	

	sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG / accScale;	/*单位 g(9.8m/s^2)*/
	sensors.acc.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	sensors.acc.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;

	applyAxis3fLpf(accLpf, &sensors.acc);
}
/*传感器任务*/
void sensorsTask(void *param)
{
	sensorsInit();	/*传感器初始化*/
	vTaskDelay(150);
	sensorsSetupSlaveRead();/*设置传感器从模式读取*/

	while (1)
	{
		if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
		{
			/*确定数据长度*/
			u8 dataLen = (u8) (SENSORS_MPU6500_BUFF_LEN +
				(isBaroPresent ? SENSORS_BARO_BUFF_LEN : 0));

			i2cdevRead(I2C1_DEV, MPU6500_ADDRESS_AD0_HIGH, MPU6500_RA_ACCEL_XOUT_H, dataLen, buffer);
			
			/*处理原始数据，并放入数据队列中*/
			processAccGyroMeasurements(&(buffer[0]));
			if (isBaroPresent)
			{
				processBarometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
			}
			
			vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
			
			sensor_data_Write(&sensors);
			xTaskResumeAll();
		}
	}	
}

void __attribute__((used)) EXTI4_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken)
	{
		portYIELD();
	}
}
/*二阶低通滤波*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (u8 i = 0; i < 3; i++) 
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
