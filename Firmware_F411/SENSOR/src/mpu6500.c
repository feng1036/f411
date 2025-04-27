#include <math.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
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

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6500_RA_SMPLRT_DIV
 */
void mpu6500SetRate(uint8_t rate)
{
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_SMPLRT_DIV, rate);
}

// CONFIG register

/** Get external FSYNC configuration.
 * Configures the external Frame Synchronization (FSYNC) pin sampling. An
 * external signal connected to the FSYNC pin can be sampled by configuring
 * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
 * strobes may be captured. The latched FSYNC signal will be sampled at the
 * Sampling Rate, as defined in register 25. After sampling, the latch will
 * reset to the current FSYNC signal state.
 *
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table.
 *
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 *
 * @return FSYNC configuration value
 */
/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6500_DLPF_BW_256
 * @see MPU6500_RA_CONFIG
 * @see MPU6500_CFG_DLPF_CFG_BIT
 * @see MPU6500_CFG_DLPF_CFG_LENGTH
 */
void mpu6500SetDLPFMode(uint8_t mode)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_CONFIG, MPU6500_CFG_DLPF_CFG_BIT,
      MPU6500_CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range id.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>/



** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6500_GYRO_FS_250
 * @see MPU6500_RA_GYRO_CONFIG
 * @see MPU6500_GCONFIG_FS_SEL_BIT
 * @see MPU6500_GCONFIG_FS_SEL_LENGTH
 */
void mpu6500SetFullScaleGyroRange(uint8_t range)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_GYRO_CONFIG, MPU6500_GCONFIG_FS_SEL_BIT,
      MPU6500_GCONFIG_FS_SEL_LENGTH, range);
}



/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>

 * Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void mpu6500SetFullScaleAccelRange(uint8_t range)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG, MPU6500_ACONFIG_AFS_SEL_BIT,
      MPU6500_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Set accelerometer digital low pass filter.
 * @param range DLPF setting
 * @see MPU6500_ACCEL_DLPF_BW_460
 * @see MPU6500_ACCEL_DLPF_BW_184
 * @see MPU6500_ACCEL_DLPF_BW_92
 * @see MPU6500_ACCEL_DLPF_BW_41
 * @see MPU6500_ACCEL_DLPF_BW_20
 * @see MPU6500_ACCEL_DLPF_BW_10
 * @see MPU6500_ACCEL_DLPF_BW_5
 */
void mpu6500SetAccelDLPF(uint8_t range)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG_2, MPU6500_ACONFIG2_DLPF_BIT,
      MPU6500_ACONFIG2_DLPF_LENGTH, range);
}

/** Get the high-pass filter configuration.
 * The DHPF is a filter module in the path leading to motion detectors (Free
 * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
 * available to the data registers (see Figure in Section 8 of the MPU-6000/
 * MPU-6500 Product Specification document).
 *
 * The high pass filter has three modes:
 *
 * <pre>
 *    Reset: The filter output settles to zero within one sample. This
 *           effectively disables the high pass filter. This mode may be toggled
 *           to quickly settle the filter.
 *
 *    On:    The high pass filter will pass signals above the cut off frequency.
 *
 *    Hold:  When triggered, the filter holds the present sample. The filter
 *           output will be the difference between the input sample and the held
 *           sample.
 * </pre>
 *
 * <pre>
 * ACCEL_HPF | Filter Mode | Cut-off Frequency
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 * </pre>
 *


** Set wait-for-external-sensor-data enabled value.
 * @param enabled New wait-for-external-sensor-data enabled value
 * @see getWaitForExternalSensorEnabled()
 * @see MPU6500_RA_I2C_MST_CTRL
 */
void mpu6500SetWaitForExternalSensorEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_I2C_MST_CTRL, MPU6500_WAIT_FOR_ES_BIT, enabled);
}

/** Set slave read/write transition enabled value.
 * @param enabled New slave read/write transition enabled value
 * @see getSlaveReadWriteTransitionEnabled()
 * @see MPU6500_RA_I2C_MST_CTRL
 */
void mpu6500SetSlaveReadWriteTransitionEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_P_NSR_BIT, enabled);
}
/** Get I2C master clock speed.
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 *
 * @return Current I2C master clock speed
 * @see MPU6500_RA_I2C_MST_CTRL
 */
/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU6500_RA_I2C_MST_CTRL
 */
void mpu6500SetMasterClockSpeed(uint8_t speed)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_CLK_BIT,
      MPU6500_I2C_MST_CLK_LENGTH, speed);
}

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6500_RA_I2C_SLV0_ADDR
 */
void mpu6500SetSlaveAddress(uint8_t num, uint8_t address)
{
  if (num > 3)
    return;
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_I2C_SLV0_ADDR + num * 3, address);
}
/** Set the active internal register for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param reg New active register for specified slave
 * @see getSlaveRegister()
 * @see MPU6500_RA_I2C_SLV0_REG
 */
void mpu6500SetSlaveRegister(uint8_t num, uint8_t reg)
{
  if (num > 3)
    return;
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_I2C_SLV0_REG + num * 3, reg);
}
/** Set the enabled value for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New enabled value for specified slave
 * @see getSlaveEnabled()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
void mpu6500SetSlaveEnabled(uint8_t num, bool enabled)
{
  if (num > 3)
    return;
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_EN_BIT,
      enabled);
}

/** Set number of bytes to read for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see getSlaveDataLength()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
void mpu6500SetSlaveDataLength(uint8_t num, uint8_t length)
{
  if (num > 3)
    return;
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_LEN_BIT,
      MPU6500_I2C_SLV_LEN_LENGTH, length);
}

/** Set the I2C address of Slave 4.
 * @param address New address for Slave 4
 * @see getSlave4Address()
 * @see MPU6500_RA_I2C_SLV4_ADDR
 */
void mpu6500SetSlave4Address(uint8_t address)
{
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_I2C_SLV4_ADDR, address);
}

/** Set Slave 4 master delay value.
 * @param delay New Slave 4 master delay value
 * @see getSlave4MasterDelay()
 * @see MPU6500_RA_I2C_SLV4_CTRL
 */
void mpu6500SetSlave4MasterDelay(uint8_t delay)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLV4_MST_DLY_BIT,
      MPU6500_I2C_SLV4_MST_DLY_LENGTH, delay);
}

/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_INT_LEVEL_BIT
 */
void mpu6500SetInterruptMode(bool mode)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_LEVEL_BIT, mode);
}

/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_INT_OPEN_BIT
 */
void mpu6500SetInterruptDrive(bool drive)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_OPEN_BIT, drive);
}
/** Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_LATCH_INT_EN_BIT
 */
void mpu6500SetInterruptLatch(bool latch)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_LATCH_INT_EN_BIT, latch);
}
/** Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_INT_RD_CLEAR_BIT
 */
void mpu6500SetInterruptLatchClear(bool clear)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_RD_CLEAR_BIT, clear);
}

/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_I2C_BYPASS_EN_BIT
 */
void mpu6500SetI2CBypassEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6500_RA_INT_ENABLE
 * @see MPU6500_INTERRUPT_FF_BIT
 **/
void mpu6500SetIntEnabled(uint8_t enabled)
{
  i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_INT_ENABLE, enabled);
}

/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU6500_RA_INT_CFG
 * @see MPU6500_INTERRUPT_DATA_RDY_BIT
 */
void mpu6500SetIntDataReadyEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_INT_ENABLE, MPU6500_INTERRUPT_DATA_RDY_BIT, enabled);
}


/** Set slave delay enabled status.
 * @param num Slave number (0-4)
 * @param enabled New slave delay enabled status.
 * @see MPU6500_RA_I2C_MST_DELAY_CTRL
 * @see MPU6500_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
void mpu6500SetSlaveDelayEnabled(uint8_t num, bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6500_RA_USER_CTRL
 * @see MPU6500_USERCTRL_I2C_MST_EN_BIT
 */
void mpu6500SetI2CMasterModeEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_I2C_MST_EN_BIT, enabled);
}

// PWR_MGMT_1 register

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6500_RA_PWR_MGMT_1
 * @see MPU6500_PWR1_DEVICE_RESET_BIT
 */
void mpu6500Reset()
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_DEVICE_RESET_BIT, 1);
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6500_RA_PWR_MGMT_1
 * @see MPU6500_PWR1_SLEEP_BIT
 */
void mpu6500SetSleepEnabled(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_SLEEP_BIT, enabled);
}

/** Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param enabled New temperature sensor enabled status
 * @see getTempSensorEnabled()
 * @see MPU6500_RA_PWR_MGMT_1
 * @see MPU6500_PWR1_TEMP_DIS_BIT
 */
void mpu6500SetTempSensorEnabled(bool enabled)
{
  // 1 is actually disabled here
  i2cdevWriteBit(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_TEMP_DIS_BIT, !enabled);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6500_RA_PWR_MGMT_1
 * @see MPU6500_PWR1_CLKSEL_BIT
 * @see MPU6500_PWR1_CLKSEL_LENGTH
 */
void mpu6500SetClockSource(uint8_t source)
{
  i2cdevWriteBits(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_CLKSEL_BIT,
      MPU6500_PWR1_CLKSEL_LENGTH, source);
}
