#ifndef __MPU6500_H
#define __MPU6500_H
#include "stm32f4xx.h"
#include "i2cdev.h"

#define	MPU6500_ADDR	(0x69)
#define MPU6500_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6500_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6500_DEFAULT_ADDRESS     MPU6500_ADDRESS_AD0_HIGH

#define MPU6500_RA_SMPLRT_DIV       0x19
#define MPU6500_RA_CONFIG           0x1A
#define MPU6500_RA_GYRO_CONFIG      0x1B
#define MPU6500_RA_ACCEL_CONFIG     0x1C
#define MPU6500_RA_ACCEL_CONFIG_2   0x1D

#define MPU6500_RA_I2C_MST_CTRL     0x24
#define MPU6500_RA_I2C_SLV0_ADDR    0x25
#define MPU6500_RA_I2C_SLV0_REG     0x26
#define MPU6500_RA_I2C_SLV0_CTRL    0x27

#define MPU6500_RA_I2C_SLV4_ADDR    0x31

#define MPU6500_RA_I2C_SLV4_CTRL    0x34

#define MPU6500_RA_INT_PIN_CFG      0x37
#define MPU6500_RA_INT_ENABLE       0x38

#define MPU6500_RA_ACCEL_XOUT_H     0x3B

#define MPU6500_RA_I2C_MST_DELAY_CTRL   0x67

#define MPU6500_RA_USER_CTRL        0x6A
#define MPU6500_RA_PWR_MGMT_1       0x6B

#define MPU6500_CFG_DLPF_CFG_BIT    2
#define MPU6500_CFG_DLPF_CFG_LENGTH 3

#define MPU6500_DLPF_BW_98          0x02

#define MPU6500_GCONFIG_FS_SEL_BIT      4
#define MPU6500_GCONFIG_FS_SEL_LENGTH   2

#define MPU6500_GYRO_FS_2000        0x03

#define MPU6500_ACONFIG_AFS_SEL_BIT         4
#define MPU6500_ACONFIG_AFS_SEL_LENGTH      2

#define MPU6500_ACONFIG2_DLPF_BIT           0
#define MPU6500_ACONFIG2_DLPF_LENGTH        2

#define MPU6500_ACCEL_DLPF_BW_41    0x03

#define MPU6500_ACCEL_FS_16         0x03

#define MPU6500_WAIT_FOR_ES_BIT     6

#define MPU6500_I2C_MST_P_NSR_BIT   4
#define MPU6500_I2C_MST_CLK_BIT     3
#define MPU6500_I2C_MST_CLK_LENGTH  4

#define MPU6500_I2C_SLV_EN_BIT      7

#define MPU6500_I2C_SLV_LEN_BIT     3
#define MPU6500_I2C_SLV_LEN_LENGTH  4

#define MPU6500_I2C_SLV4_MST_DLY_BIT    4
#define MPU6500_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6500_INTCFG_INT_LEVEL_BIT        7
#define MPU6500_INTCFG_INT_OPEN_BIT         6
#define MPU6500_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6500_INTCFG_INT_RD_CLEAR_BIT     4

#define MPU6500_INTCFG_I2C_BYPASS_EN_BIT    1

#define MPU6500_INTERRUPT_DATA_RDY_BIT      0

#define MPU6500_USERCTRL_I2C_MST_EN_BIT         5

#define MPU6500_PWR1_DEVICE_RESET_BIT   7
#define MPU6500_PWR1_SLEEP_BIT          6
#define MPU6500_PWR1_TEMP_DIS_BIT       3
#define MPU6500_PWR1_CLKSEL_BIT         2
#define MPU6500_PWR1_CLKSEL_LENGTH      3


#define MPU6500_CLOCK_PLL_XGYRO         0x01

#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6500_G_PER_LSB_16     (float)((2 * 16) / 65536.0)


void mpu6500Init(I2C_Dev *i2cPort);

void mpu6500SetSlaveAddress(uint8_t num, uint8_t address);
void mpu6500SetSlave4MasterDelay(uint8_t delay);
void mpu6500SetSlaveRegister(uint8_t num, uint8_t reg);
void mpu6500SetSlaveDataLength(uint8_t num, uint8_t length);
void mpu6500SetSlaveDelayEnabled(uint8_t num, bool enabled);
void mpu6500SetSlaveEnabled(uint8_t num, bool enabled);

void mpu6500SetClockSource(uint8_t source);
void mpu6500SetMasterClockSpeed(uint8_t speed);

// ACCEL_CONFIG2 register
void mpu6500SetAccelDLPF(uint8_t range);
void mpu6500SetRate(uint8_t rate);
void mpu6500SetDLPFMode(uint8_t mode);
// I2C_MST_CTRL register
void mpu6500SetWaitForExternalSensorEnabled(bool enabled);
void mpu6500SetSlaveReadWriteTransitionEnabled(bool enabled);

void mpu6500SetIntEnabled(uint8_t enabled);
// INT_PIN_CFG register
void mpu6500SetInterruptDrive(bool drive);
void mpu6500SetInterruptLatch(bool latch);
void mpu6500SetInterruptMode(bool mode);
void mpu6500SetInterruptLatchClear(bool clear);
void mpu6500SetI2CBypassEnabled(bool enabled);

// INT_ENABLE register
void mpu6500SetIntDataReadyEnabled(bool enabled);

// USER_CTRL register
void mpu6500SetI2CMasterModeEnabled(bool enabled);

// PWR_MGMT_1 register
void mpu6500Reset(void);
void mpu6500SetSleepEnabled(bool enabled);
void mpu6500SetTempSensorEnabled(bool enabled);

void mpu6500SetFullScaleGyroRange(uint8_t range);
void mpu6500SetFullScaleAccelRange(uint8_t range);

#endif /* __MPU6500_H */

