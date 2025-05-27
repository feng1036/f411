#ifndef __SENSORS_H
#define __SENSORS_H
#include "delay.h"
#include "mpu6500.h"
#include "bmp280.h"
#include "filter.h"
#define SENSORS_ENABLE_PRESSURE_BMP280	/*��ѹ��ʹ��bmp280*/
#define BARO_UPDATE_RATE		RATE_50_HZ
#define SENSOR9_UPDATE_RATE   	RATE_500_HZ
#define SENSOR9_UPDATE_DT     	(1.0f/SENSOR9_UPDATE_RATE)

#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_16	
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES		(1024)	/* ���㷽��Ĳ����������� */
#define GYRO_VARIANCE_BASE				(4000)	/* ��������ƫ������ֵ */
#define SENSORS_ACC_SCALE_SAMPLES  		(200)		/* ���ټƲ������� */

// MPU9250����ģʽ��ȡ���� ����������
#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_BARO_STATUS_LEN		1
#define SENSORS_BARO_DATA_LEN		6
#define SENSORS_BARO_BUFF_LEN       (SENSORS_BARO_STATUS_LEN + SENSORS_BARO_DATA_LEN)

/*��ͨ�˲�����*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30


typedef union 
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
} Axis3i16;

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

typedef struct
{
	Axis3f acc;
	Axis3f gyro;
	baro_t baro;
} sensorData_t;

typedef struct
{
	Axis3f     bias;
	bool       isBiasValueFound;
	bool       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj; 

static BiasObj	gyroBiasRunning;
static Axis3f  gyroBias;
static sensorData_t sensors;
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

// static xSemaphoreHandle sensorsDataReady;
static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};
void sensorsTask(void *param);
void sensorsInit(void);			/*��������ʼ��*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
#endif //__SENSORS_H 
