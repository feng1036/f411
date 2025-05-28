#include "i2c.h"
#include <stdbool.h>
// Device Addresses

// Function Prototypes
void MPU6500_Init(struct I2C_Dev *I2c_Port);
void Sensor_Setup_Slave_Read(void);

// Device Address
#define BMP280_I2C_ADDRESS                  (0x76)
#define BMP280_DEFAULT_CHIP_ID              (0x58)

// Calibration Data Structure
struct BMP280_Calib_Data 
{
    uint16_t Dig_T1;
    int16_t Dig_T2;
    int16_t Dig_T3;
    uint16_t Dig_P1;
    int16_t Dig_P2;
    int16_t Dig_P3;
    int16_t Dig_P4;
    int16_t Dig_P5;
    int16_t Dig_P6;
    int16_t Dig_P7;
    int16_t Dig_P8;
    int16_t Dig_P9;
    int32_t T_Fine;
};

// Function Prototypes
bool BMP280_Init(struct I2C_Dev *I2c_Port);
uint32_t BMP280_Compensate_T(int32_t Adc_T);
uint32_t BMP280_Compensate_P(int32_t Adc_P);
float BMP280_Pressure_To_Altitude(float *Pressure);
