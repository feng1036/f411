#include <math.h>
#include "stm32f4xx.h"
#include "i2cdev.h"
#include "mpu6500.h"
static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

void mpu6500Init(I2C_Dev *i2cPort)
{
  if (isInit)
    return;

  I2Cx = i2cPort;
  devAddr = MPU6500_ADDRESS_AD0_HIGH;

  isInit = true;
}

///** Set gyroscope sample rate divider.
void mpu6500SetRate(uint8_t rate)
{
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_SMPLRT_DIV, rate);
}

// CONFIG register
void mpu6500SetDLPFMode(uint8_t mode)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_CONFIG, MPU6500_CFG_DLPF_CFG_BIT,
      MPU6500_CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register
void mpu6500SetFullScaleGyroRange(uint8_t range)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_GYRO_CONFIG, MPU6500_GCONFIG_FS_SEL_BIT,
      MPU6500_GCONFIG_FS_SEL_LENGTH, range);
}



void mpu6500SetFullScaleAccelRange(uint8_t range)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG, MPU6500_ACONFIG_AFS_SEL_BIT,
      MPU6500_ACONFIG_AFS_SEL_LENGTH, range);
}


void mpu6500SetAccelDLPF(uint8_t range)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG_2, MPU6500_ACONFIG2_DLPF_BIT,
      MPU6500_ACONFIG2_DLPF_LENGTH, range);
}

void mpu6500SetWaitForExternalSensorEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_I2C_MST_CTRL, MPU6500_WAIT_FOR_ES_BIT, enabled);
}

void mpu6500SetSlaveReadWriteTransitionEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_P_NSR_BIT, enabled);
}

void mpu6500SetMasterClockSpeed(uint8_t speed)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_CLK_BIT,
      MPU6500_I2C_MST_CLK_LENGTH, speed);
}


void mpu6500SetSlaveAddress(uint8_t num, uint8_t address)
{
  if (num > 3)
    return;
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_I2C_SLV0_ADDR + num * 3, address);
}

void mpu6500SetSlaveRegister(uint8_t num, uint8_t reg)
{
  if (num > 3)
    return;
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_I2C_SLV0_REG + num * 3, reg);
}

void mpu6500SetSlaveEnabled(uint8_t num, bool enabled)
{
  if (num > 3)
    return;
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_EN_BIT,
      enabled);
}

void mpu6500SetSlaveDataLength(uint8_t num, uint8_t length)
{
  if (num > 3)
    return;
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_LEN_BIT,
      MPU6500_I2C_SLV_LEN_LENGTH, length);
}

void mpu6500SetSlave4Address(uint8_t address)
{
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_I2C_SLV4_ADDR, address);
}

void mpu6500SetSlave4MasterDelay(uint8_t delay)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLV4_MST_DLY_BIT,
      MPU6500_I2C_SLV4_MST_DLY_LENGTH, delay);
}

void mpu6500SetInterruptMode(bool mode)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_LEVEL_BIT, mode);
}

void mpu6500SetInterruptDrive(bool drive)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_OPEN_BIT, drive);
}

void mpu6500SetInterruptLatch(bool latch)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_LATCH_INT_EN_BIT, latch);
}

void mpu6500SetInterruptLatchClear(bool clear)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_RD_CLEAR_BIT, clear);
}


void mpu6500SetI2CBypassEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void mpu6500SetIntEnabled(uint8_t enabled)
{
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_INT_ENABLE, enabled);
}

void mpu6500SetIntDataReadyEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_ENABLE, MPU6500_INTERRUPT_DATA_RDY_BIT, enabled);
}


void mpu6500SetSlaveDelayEnabled(uint8_t num, bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

void mpu6500SetI2CMasterModeEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_I2C_MST_EN_BIT, enabled);
}

// PWR_MGMT_1 register
void mpu6500Reset()
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_DEVICE_RESET_BIT, 1);
}

void mpu6500SetSleepEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_SLEEP_BIT, enabled);
}

void mpu6500SetTempSensorEnabled(bool enabled)
{
  // 1 is actually disabled here
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_TEMP_DIS_BIT, !enabled);
}

void mpu6500SetClockSource(uint8_t source)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_CLKSEL_BIT,
      MPU6500_PWR1_CLKSEL_LENGTH, source);
}
