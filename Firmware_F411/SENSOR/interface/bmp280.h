#ifndef __BMP280_H
#define __BMP280_H
#include "stm32f4xx.h"
#include "i2cdev.h"

#define BMP280_I2C_ADDR					(0x76)
#define BMP280_DEFAULT_CHIP_ID			(0x58)

#define BMP280_CHIP_ID					(0xD0)  /* Chip ID Register */
#define BMP280_STAT_REG					(0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG			(0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG				(0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG			(0xF7)  /* Pressure MSB Register */

#define BMP280_NORMAL_MODE				(0x03)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)

#define BMP280_OVERSAMP_8X				(0x04)

bool bmp280Init(I2C_Dev *i2cPort);
u32 bmp280CompensateT(s32 adcT);
u32 bmp280CompensateP(s32 adcP);
float bmp280PressureToAltitude(float* pressure/*, float* groundPressure, float* groundTemp*/);
#endif
