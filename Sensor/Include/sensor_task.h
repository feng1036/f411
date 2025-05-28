#ifndef __SENSOR_TASK_H
#define __SENSORS_TASK_H
#include <stdbool.h>
#define SENSOR9_UPDATE_RATE                 RATE_500_HZ
#define SENSOR9_UPDATE_DT                   (1.0f/SENSOR9_UPDATE_RATE)
#define SENSORS_DEG_PER_LSB_CFG             MPU6500_DEG_PER_LSB_2000
#define SENSORS_G_PER_LSB_CFG               MPU6500_G_PER_LSB_16
#define SENSORS_NBR_OF_BIAS_SAMPLES         (1024)    /* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE                  (4000)    /* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES           (200)        /* 加速计采样个数 */
// MPU9250主机模式读取数据 缓冲区长度
#define SENSORS_MPU6500_BUFF_LEN            (14)
#define SENSORS_BARO_STATUS_LEN             (1)
#define SENSORS_BARO_DATA_LEN               (6)
#define SENSORS_BARO_BUFF_LEN               (SENSORS_BARO_STATUS_LEN+SENSORS_BARO_DATA_LEN)


struct Axis3i
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Axis3f
{
    float x;
    float y;
    float z;
};

struct Baro
{
    float Pressure;
    float Temperature;
    float Asl;
};

struct Sensor_Data
{
    Axis3f Acc;
    Axis3f Gyro;
    Baro Baro;
};

struct Bias_Obj
{
    Axis3f     Bias;
    bool       Is_Bias_Value_Found;
    bool       Is_Buffer_Filled;
    Axis3i16*  Buffer_Head;
    Axis3i16   Buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
};

static Bias_Obj    Gyro_Bias_Running;
static Axis3f     Gyro_Bias;
static Sensor_Data Sensor;
static Lpf2p_Data AccLpf[3];
static Lpf2p_Data GyroLpf[3];

static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN+SENSORS_BARO_BUFF_LEN];
void Sensor_Task(void *param);
void Sensor_Init(void);/*传感器初始化*/
static void Apply_Axis3flpf(lpf2pData *data, Axis3f* in);
static void Sensor_Bias_Obj_Init(BiasObj* bias);
static void Sensors_Calculate_VarianceAndMean(Bias_Obj* Bias, Axis3f* Var_Out, Axis3f* MeanOut);
static bool sensorsFindBiasValue(Bias_Obj* Bias);
static void sensorsAddBiasValue(Bias_Obj* Bias, int16_t x, int16_t y, int16_t z);
#endif //__SENSORS_H
