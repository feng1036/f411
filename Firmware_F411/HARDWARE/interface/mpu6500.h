#ifndef __MPU6500_H
#define __MPU6500_H
#include "stm32f4xx.h"
#include "i2cdev.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * MPU6500驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define	MPU6500_ADDR	(0x69)
#define MPU6500_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6500_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6500_DEFAULT_ADDRESS     MPU6500_ADDRESS_AD0_HIGH

//Product ID Description for MPU6500:  | High 4 bits  | Low 4 bits        |
//                                     | Product Name | Product Revision  |
#define MPU6500_REV_C4_ES     0x14  //        0001           0100
#define MPU6500_REV_C5_ES     0x15  //        0001           0101
#define MPU6500_REV_D6_ES     0x16  //        0001           0110
#define MPU6500_REV_D7_ES     0x17  //        0001           0111
#define MPU6500_REV_D8_ES     0x18  //        0001           1000
#define MPU6500_REV_C4        0x54  //        0101           0100
#define MPU6500_REV_C5        0x55  //        0101           0101
#define MPU6500_REV_D6        0x56  //        0101           0110
#define MPU6500_REV_D7        0x57  //        0101           0111
#define MPU6500_REV_D8        0x58  //        0101           1000
#define MPU6500_REV_D9        0x59  //        0101           1001

#define MPU6500_RA_ST_X_GYRO        0x00
#define MPU6500_RA_ST_Y_GYRO        0x01
#define MPU6500_RA_ST_Z_GYRO        0x02
#define MPU6500_RA_ST_X_ACCEL       0x0D
#define MPU6500_RA_ST_Y_ACCEL       0x0E
#define MPU6500_RA_ST_Z_ACCEL       0x0F
#define MPU6500_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6500_RA_XG_OFFS_USRL     0x14
#define MPU6500_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6500_RA_YG_OFFS_USRL     0x16
#define MPU6500_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6500_RA_ZG_OFFS_USRL     0x18
#define MPU6500_RA_SMPLRT_DIV       0x19
#define MPU6500_RA_CONFIG           0x1A
#define MPU6500_RA_GYRO_CONFIG      0x1B
#define MPU6500_RA_ACCEL_CONFIG     0x1C
#define MPU6500_RA_ACCEL_CONFIG_2   0x1D
#define MPU6500_RA_LP_ACCEL_ODR     0x1E
#define MPU6500_RA_WOM_THR          0x1F

#define MPU6500_RA_FIFO_EN          0x23
#define MPU6500_RA_I2C_MST_CTRL     0x24
#define MPU6500_RA_I2C_SLV0_ADDR    0x25
#define MPU6500_RA_I2C_SLV0_REG     0x26
#define MPU6500_RA_I2C_SLV0_CTRL    0x27
#define MPU6500_RA_I2C_SLV1_ADDR    0x28
#define MPU6500_RA_I2C_SLV1_REG     0x29
#define MPU6500_RA_I2C_SLV1_CTRL    0x2A
#define MPU6500_RA_I2C_SLV2_ADDR    0x2B
#define MPU6500_RA_I2C_SLV2_REG     0x2C
#define MPU6500_RA_I2C_SLV2_CTRL    0x2D
#define MPU6500_RA_I2C_SLV3_ADDR    0x2E
#define MPU6500_RA_I2C_SLV3_REG     0x2F
#define MPU6500_RA_I2C_SLV3_CTRL    0x30
#define MPU6500_RA_I2C_SLV4_ADDR    0x31
#define MPU6500_RA_I2C_SLV4_REG     0x32
#define MPU6500_RA_I2C_SLV4_DO      0x33
#define MPU6500_RA_I2C_SLV4_CTRL    0x34
#define MPU6500_RA_I2C_SLV4_DI      0x35
#define MPU6500_RA_I2C_MST_STATUS   0x36
#define MPU6500_RA_INT_PIN_CFG      0x37
#define MPU6500_RA_INT_ENABLE       0x38
#define MPU6500_RA_DMP_INT_STATUS   0x39
#define MPU6500_RA_INT_STATUS       0x3A
#define MPU6500_RA_ACCEL_XOUT_H     0x3B
#define MPU6500_RA_ACCEL_XOUT_L     0x3C
#define MPU6500_RA_ACCEL_YOUT_H     0x3D
#define MPU6500_RA_ACCEL_YOUT_L     0x3E
#define MPU6500_RA_ACCEL_ZOUT_H     0x3F
#define MPU6500_RA_ACCEL_ZOUT_L     0x40
#define MPU6500_RA_TEMP_OUT_H       0x41
#define MPU6500_RA_TEMP_OUT_L       0x42
#define MPU6500_RA_GYRO_XOUT_H      0x43
#define MPU6500_RA_GYRO_XOUT_L      0x44
#define MPU6500_RA_GYRO_YOUT_H      0x45
#define MPU6500_RA_GYRO_YOUT_L      0x46
#define MPU6500_RA_GYRO_ZOUT_H      0x47
#define MPU6500_RA_GYRO_ZOUT_L      0x48
#define MPU6500_RA_EXT_SENS_DATA_00 0x49
#define MPU6500_RA_EXT_SENS_DATA_01 0x4A
#define MPU6500_RA_EXT_SENS_DATA_02 0x4B
#define MPU6500_RA_EXT_SENS_DATA_03 0x4C
#define MPU6500_RA_EXT_SENS_DATA_04 0x4D
#define MPU6500_RA_EXT_SENS_DATA_05 0x4E
#define MPU6500_RA_EXT_SENS_DATA_06 0x4F
#define MPU6500_RA_EXT_SENS_DATA_07 0x50
#define MPU6500_RA_EXT_SENS_DATA_08 0x51
#define MPU6500_RA_EXT_SENS_DATA_09 0x52
#define MPU6500_RA_EXT_SENS_DATA_10 0x53
#define MPU6500_RA_EXT_SENS_DATA_11 0x54
#define MPU6500_RA_EXT_SENS_DATA_12 0x55
#define MPU6500_RA_EXT_SENS_DATA_13 0x56
#define MPU6500_RA_EXT_SENS_DATA_14 0x57
#define MPU6500_RA_EXT_SENS_DATA_15 0x58
#define MPU6500_RA_EXT_SENS_DATA_16 0x59
#define MPU6500_RA_EXT_SENS_DATA_17 0x5A
#define MPU6500_RA_EXT_SENS_DATA_18 0x5B
#define MPU6500_RA_EXT_SENS_DATA_19 0x5C
#define MPU6500_RA_EXT_SENS_DATA_20 0x5D
#define MPU6500_RA_EXT_SENS_DATA_21 0x5E
#define MPU6500_RA_EXT_SENS_DATA_22 0x5F
#define MPU6500_RA_EXT_SENS_DATA_23 0x60
#define MPU6500_RA_MOT_DETECT_STATUS    0x61
#define MPU6500_RA_I2C_SLV0_DO      0x63
#define MPU6500_RA_I2C_SLV1_DO      0x64
#define MPU6500_RA_I2C_SLV2_DO      0x65
#define MPU6500_RA_I2C_SLV3_DO      0x66
#define MPU6500_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6500_RA_SIGNAL_PATH_RESET    0x68
#define MPU6500_RA_MOT_DETECT_CTRL      0x69
#define MPU6500_RA_USER_CTRL        0x6A
#define MPU6500_RA_PWR_MGMT_1       0x6B
#define MPU6500_RA_PWR_MGMT_2       0x6C
#define MPU6500_RA_BANK_SEL         0x6D
#define MPU6500_RA_MEM_START_ADDR   0x6E
#define MPU6500_RA_MEM_R_W          0x6F
#define MPU6500_RA_DMP_CFG_1        0x70
#define MPU6500_RA_DMP_CFG_2        0x71
#define MPU6500_RA_FIFO_COUNTH      0x72
#define MPU6500_RA_FIFO_COUNTL      0x73
#define MPU6500_RA_FIFO_R_W         0x74
#define MPU6500_RA_WHO_AM_I         0x75

#define MPU6500_RA_XA_OFFSET_H      0x77
#define MPU6500_RA_XA_OFFSET_L      0x78
#define MPU6500_RA_YA_OFFSET_H      0x7A
#define MPU6500_RA_YA_OFFSET_L      0x7B
#define MPU6500_RA_ZA_OFFSET_H      0x7D
#define MPU6500_RA_ZA_OFFSET_L      0x7E

#define MPU6500_TC_PWR_MODE_BIT     7
#define MPU6500_TC_OFFSET_BIT       6
#define MPU6500_TC_OFFSET_LENGTH    6
#define MPU6500_TC_OTP_BNK_VLD_BIT  0

#define MPU6500_VDDIO_LEVEL_VLOGIC  0
#define MPU6500_VDDIO_LEVEL_VDD     1

#define MPU6500_CFG_EXT_SYNC_SET_BIT    5
#define MPU6500_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6500_CFG_DLPF_CFG_BIT    2
#define MPU6500_CFG_DLPF_CFG_LENGTH 3

#define MPU6500_EXT_SYNC_DISABLED       0x0
#define MPU6500_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6500_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6500_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6500_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6500_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6500_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6500_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6500_DLPF_BW_256         0x00
#define MPU6500_DLPF_BW_188         0x01
#define MPU6500_DLPF_BW_98          0x02
#define MPU6500_DLPF_BW_42          0x03
#define MPU6500_DLPF_BW_20          0x04
#define MPU6500_DLPF_BW_10          0x05
#define MPU6500_DLPF_BW_5           0x06

#define MPU6500_GCONFIG_XG_ST_BIT       7
#define MPU6500_GCONFIG_YG_ST_BIT       6
#define MPU6500_GCONFIG_ZG_ST_BIT       5
#define MPU6500_GCONFIG_FS_SEL_BIT      4
#define MPU6500_GCONFIG_FS_SEL_LENGTH   2


#define MPU6500_GYRO_FS_250         0x00
#define MPU6500_GYRO_FS_500         0x01
#define MPU6500_GYRO_FS_1000        0x02
#define MPU6500_GYRO_FS_2000        0x03

#define MPU6500_ACONFIG_XA_ST_BIT           7
#define MPU6500_ACONFIG_YA_ST_BIT           6
#define MPU6500_ACONFIG_ZA_ST_BIT           5
#define MPU6500_ACONFIG_AFS_SEL_BIT         4
#define MPU6500_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6500_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6500_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6500_ACONFIG2_FCHOICE_B_BIT      2
#define MPU6500_ACONFIG2_FCHOICE_B_LENGTH   2
#define MPU6500_ACONFIG2_DLPF_BIT           0
#define MPU6500_ACONFIG2_DLPF_LENGTH        2

#define MPU6500_ACCEL_DLPF_BW_460   0x00
#define MPU6500_ACCEL_DLPF_BW_184   0x01
#define MPU6500_ACCEL_DLPF_BW_92    0x02
#define MPU6500_ACCEL_DLPF_BW_41    0x03
#define MPU6500_ACCEL_DLPF_BW_20    0x04
#define MPU6500_ACCEL_DLPF_BW_10    0x05
#define MPU6500_ACCEL_DLPF_BW_5     0x06

#define MPU6500_ACCEL_FS_2          0x00
#define MPU6500_ACCEL_FS_4          0x01
#define MPU6500_ACCEL_FS_8          0x02
#define MPU6500_ACCEL_FS_16         0x03

#define MPU6500_DHPF_RESET          0x00
#define MPU6500_DHPF_5              0x01
#define MPU6500_DHPF_2P5            0x02
#define MPU6500_DHPF_1P25           0x03
#define MPU6500_DHPF_0P63           0x04
#define MPU6500_DHPF_HOLD           0x07

#define MPU6500_TEMP_FIFO_EN_BIT    7
#define MPU6500_XG_FIFO_EN_BIT      6
#define MPU6500_YG_FIFO_EN_BIT      5
#define MPU6500_ZG_FIFO_EN_BIT      4
#define MPU6500_ACCEL_FIFO_EN_BIT   3
#define MPU6500_SLV2_FIFO_EN_BIT    2
#define MPU6500_SLV1_FIFO_EN_BIT    1
#define MPU6500_SLV0_FIFO_EN_BIT    0

#define MPU6500_MULT_MST_EN_BIT     7
#define MPU6500_WAIT_FOR_ES_BIT     6
#define MPU6500_SLV_3_FIFO_EN_BIT   5
#define MPU6500_I2C_MST_P_NSR_BIT   4
#define MPU6500_I2C_MST_CLK_BIT     3
#define MPU6500_I2C_MST_CLK_LENGTH  4

#define MPU6500_CLOCK_DIV_348       0x0
#define MPU6500_CLOCK_DIV_333       0x1
#define MPU6500_CLOCK_DIV_320       0x2
#define MPU6500_CLOCK_DIV_308       0x3
#define MPU6500_CLOCK_DIV_296       0x4
#define MPU6500_CLOCK_DIV_286       0x5
#define MPU6500_CLOCK_DIV_276       0x6
#define MPU6500_CLOCK_DIV_267       0x7
#define MPU6500_CLOCK_DIV_258       0x8
#define MPU6500_CLOCK_DIV_500       0x9
#define MPU6500_CLOCK_DIV_471       0xA
#define MPU6500_CLOCK_DIV_444       0xB
#define MPU6500_CLOCK_DIV_421       0xC
#define MPU6500_CLOCK_DIV_400       0xD
#define MPU6500_CLOCK_DIV_381       0xE
#define MPU6500_CLOCK_DIV_364       0xF

#define MPU6500_I2C_SLV_RW_BIT      7
#define MPU6500_I2C_SLV_ADDR_BIT    6
#define MPU6500_I2C_SLV_ADDR_LENGTH 7
#define MPU6500_I2C_SLV_EN_BIT      7
#define MPU6500_I2C_SLV_BYTE_SW_BIT 6
#define MPU6500_I2C_SLV_REG_DIS_BIT 5
#define MPU6500_I2C_SLV_GRP_BIT     4
#define MPU6500_I2C_SLV_LEN_BIT     3
#define MPU6500_I2C_SLV_LEN_LENGTH  4

#define MPU6500_I2C_SLV4_RW_BIT         7
#define MPU6500_I2C_SLV4_ADDR_BIT       6
#define MPU6500_I2C_SLV4_ADDR_LENGTH    7
#define MPU6500_I2C_SLV4_EN_BIT         7
#define MPU6500_I2C_SLV4_INT_EN_BIT     6
#define MPU6500_I2C_SLV4_REG_DIS_BIT    5
#define MPU6500_I2C_SLV4_MST_DLY_BIT    4
#define MPU6500_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6500_MST_PASS_THROUGH_BIT    7
#define MPU6500_MST_I2C_SLV4_DONE_BIT   6
#define MPU6500_MST_I2C_LOST_ARB_BIT    5
#define MPU6500_MST_I2C_SLV4_NACK_BIT   4
#define MPU6500_MST_I2C_SLV3_NACK_BIT   3
#define MPU6500_MST_I2C_SLV2_NACK_BIT   2
#define MPU6500_MST_I2C_SLV1_NACK_BIT   1
#define MPU6500_MST_I2C_SLV0_NACK_BIT   0

#define MPU6500_INTCFG_INT_LEVEL_BIT        7
#define MPU6500_INTCFG_INT_OPEN_BIT         6
#define MPU6500_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6500_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6500_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6500_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6500_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6500_INTCFG_CLKOUT_EN_BIT        0

#define MPU6500_INTMODE_ACTIVEHIGH  0x00
#define MPU6500_INTMODE_ACTIVELOW   0x01

#define MPU6500_INTDRV_PUSHPULL     0x00
#define MPU6500_INTDRV_OPENDRAIN    0x01

#define MPU6500_INTLATCH_50USPULSE  0x00
#define MPU6500_INTLATCH_WAITCLEAR  0x01

#define MPU6500_INTCLEAR_STATUSREAD 0x00
#define MPU6500_INTCLEAR_ANYREAD    0x01

#define MPU6500_INTERRUPT_FF_BIT            7
#define MPU6500_INTERRUPT_MOT_BIT           6
#define MPU6500_INTERRUPT_ZMOT_BIT          5
#define MPU6500_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6500_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6500_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6500_INTERRUPT_DMP_INT_BIT       1
#define MPU6500_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6500_DMPINT_5_BIT            5
#define MPU6500_DMPINT_4_BIT            4
#define MPU6500_DMPINT_3_BIT            3
#define MPU6500_DMPINT_2_BIT            2
#define MPU6500_DMPINT_1_BIT            1
#define MPU6500_DMPINT_0_BIT            0

#define MPU6500_MOTION_MOT_XNEG_BIT     7
#define MPU6500_MOTION_MOT_XPOS_BIT     6
#define MPU6500_MOTION_MOT_YNEG_BIT     5
#define MPU6500_MOTION_MOT_YPOS_BIT     4
#define MPU6500_MOTION_MOT_ZNEG_BIT     3
#define MPU6500_MOTION_MOT_ZPOS_BIT     2
#define MPU6500_MOTION_MOT_ZRMOT_BIT    0

#define MPU6500_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6500_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6500_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6500_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6500_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6500_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6500_PATHRESET_GYRO_RESET_BIT    2
#define MPU6500_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6500_PATHRESET_TEMP_RESET_BIT    0

#define MPU6500_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6500_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6500_DETECT_FF_COUNT_BIT             3
#define MPU6500_DETECT_FF_COUNT_LENGTH          2
#define MPU6500_DETECT_MOT_COUNT_BIT            1
#define MPU6500_DETECT_MOT_COUNT_LENGTH         2

#define MPU6500_DETECT_DECREMENT_RESET  0x0
#define MPU6500_DETECT_DECREMENT_1      0x1
#define MPU6500_DETECT_DECREMENT_2      0x2
#define MPU6500_DETECT_DECREMENT_4      0x3

#define MPU6500_USERCTRL_DMP_EN_BIT             7
#define MPU6500_USERCTRL_FIFO_EN_BIT            6
#define MPU6500_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6500_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6500_USERCTRL_DMP_RESET_BIT          3
#define MPU6500_USERCTRL_FIFO_RESET_BIT         2
#define MPU6500_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6500_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6500_PWR1_DEVICE_RESET_BIT   7
#define MPU6500_PWR1_SLEEP_BIT          6
#define MPU6500_PWR1_CYCLE_BIT          5
#define MPU6500_PWR1_TEMP_DIS_BIT       3
#define MPU6500_PWR1_CLKSEL_BIT         2
#define MPU6500_PWR1_CLKSEL_LENGTH      3

#define MPU6500_CLOCK_INTERNAL          0x00
#define MPU6500_CLOCK_PLL_XGYRO         0x01
#define MPU6500_CLOCK_PLL_YGYRO         0x02
#define MPU6500_CLOCK_PLL_ZGYRO         0x03
#define MPU6500_CLOCK_PLL_EXT32K        0x04
#define MPU6500_CLOCK_PLL_EXT19M        0x05
#define MPU6500_CLOCK_KEEP_RESET        0x07

#define MPU6500_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6500_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6500_PWR2_STBY_XA_BIT            5
#define MPU6500_PWR2_STBY_YA_BIT            4
#define MPU6500_PWR2_STBY_ZA_BIT            3
#define MPU6500_PWR2_STBY_XG_BIT            2
#define MPU6500_PWR2_STBY_YG_BIT            1
#define MPU6500_PWR2_STBY_ZG_BIT            0

#define MPU6500_WAKE_FREQ_1P25      0x0
#define MPU6500_WAKE_FREQ_2P5       0x1
#define MPU6500_WAKE_FREQ_5         0x2
#define MPU6500_WAKE_FREQ_10        0x3

#define MPU6500_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6500_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6500_BANKSEL_MEM_SEL_BIT         4
#define MPU6500_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6500_WHO_AM_I_BIT        6
#define MPU6500_WHO_AM_I_LENGTH     6

#define MPU6500_DMP_MEMORY_BANKS        8
#define MPU6500_DMP_MEMORY_BANK_SIZE    256
#define MPU6500_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6500_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6500_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6500_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6500_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6500_G_PER_LSB_16     (float)((2 * 16) / 65536.0)

// Test limits
#define MPU6500_ST_GYRO_LOW      (-14.0)  // %
#define MPU6500_ST_GYRO_HIGH     14.0  // %
#define MPU6500_ST_ACCEL_LOW     (-14.0)  // %
#define MPU6500_ST_ACCEL_HIGH    14.0  // %


void mpu6500Init(I2C_Dev *i2cPort);
bool mpu6500Test(void);

bool mpu6500TestConnection(void);
bool mpu6500EvaluateSelfTest(float low, float high, float value, char* string);
bool mpu6500SelfTest(void);


// AUX_VDDIO register
u8 mpu6500GetAuxVDDIOLevel(void);
void mpu6500SetAuxVDDIOLevel(u8 level);

// SMPLRT_DIV register
u8 mpu6500GetRate(void);
void mpu6500SetRate(u8 rate);

// CONFIG register
u8 mpu6500GetExternalFrameSync(void);
void mpu6500SetExternalFrameSync(u8 sync);
u8 mpu6500GetDLPFMode(void);
void mpu6500SetDLPFMode(u8 bandwidth);

// GYRO_CONFIG register
void mpu6500SetGyroXSelfTest(bool enabled);
void mpu6500SetGyroYSelfTest(bool enabled);
void mpu6500SetGyroZSelfTest(bool enabled);
u8 mpu6500GetFullScaleGyroRangeId(void);
float mpu6500GetFullScaleGyroDPL(void);
void mpu6500SetFullScaleGyroRange(u8 range);

// ACCEL_CONFIG register
bool mpu6500GetAccelXSelfTest(void);
void mpu6500SetAccelXSelfTest(bool enabled);
bool mpu6500GetAccelYSelfTest(void);
void mpu6500SetAccelYSelfTest(bool enabled);
bool mpu6500GetAccelZSelfTest(void);
void mpu6500SetAccelZSelfTest(bool enabled);
u8 mpu6500GetFullScaleAccelRangeId(void);
void mpu6500SetFullScaleAccelRange(u8 range);
float mpu6500GetFullScaleAccelGPL(void);
u8 mpu6500GetDHPFMode(void);
void mpu6500SetDHPFMode(u8 mode);

// ACCEL_CONFIG2 register
void mpu6500SetAccelDLPF(uint8_t range);

// FF_THR register
u8 mpu6500GetFreefallDetectionThreshold(void);
void mpu6500SetFreefallDetectionThreshold(u8 threshold);

// FF_DUR register
u8 mpu6500GetFreefallDetectionDuration(void);
void mpu6500SetFreefallDetectionDuration(u8 duration);

// MOT_THR register
u8 mpu6500GetMotionDetectionThreshold(void);
void mpu6500SetMotionDetectionThreshold(u8 threshold);

// MOT_DUR register
u8 mpu6500GetMotionDetectionDuration(void);
void mpu6500SetMotionDetectionDuration(u8 duration);

// ZRMOT_THR register
u8 mpu6500GetZeroMotionDetectionThreshold(void);
void mpu6500SetZeroMotionDetectionThreshold(u8 threshold);

// ZRMOT_DUR register
u8 mpu6500GetZeroMotionDetectionDuration(void);
void mpu6500SetZeroMotionDetectionDuration(u8 duration);

// FIFO_EN register
bool mpu6500GetTempFIFOEnabled(void);
void mpu6500SetTempFIFOEnabled(bool enabled);
bool mpu6500GetXGyroFIFOEnabled(void);
void mpu6500SetXGyroFIFOEnabled(bool enabled);
bool mpu6500GetYGyroFIFOEnabled(void);
void mpu6500SetYGyroFIFOEnabled(bool enabled);
bool mpu6500GetZGyroFIFOEnabled(void);
void mpu6500SetZGyroFIFOEnabled(bool enabled);
bool mpu6500GetAccelFIFOEnabled(void);
void mpu6500SetAccelFIFOEnabled(bool enabled);
bool mpu6500GetSlave2FIFOEnabled(void);
void mpu6500SetSlave2FIFOEnabled(bool enabled);
bool mpu6500GetSlave1FIFOEnabled(void);
void mpu6500SetSlave1FIFOEnabled(bool enabled);
bool mpu6500GetSlave0FIFOEnabled(void);
void mpu6500SetSlave0FIFOEnabled(bool enabled);

// I2C_MST_CTRL register
bool mpu6500GetMultiMasterEnabled(void);
void mpu6500SetMultiMasterEnabled(bool enabled);
bool mpu6500GetWaitForExternalSensorEnabled(void);
void mpu6500SetWaitForExternalSensorEnabled(bool enabled);
bool mpu6500GetSlave3FIFOEnabled(void);
void mpu6500SetSlave3FIFOEnabled(bool enabled);
bool mpu6500GetSlaveReadWriteTransitionEnabled(void);
void mpu6500SetSlaveReadWriteTransitionEnabled(bool enabled);
u8 mpu6500GetMasterClockSpeed(void);
void mpu6500SetMasterClockSpeed(u8 speed);

// I2C_SLV* registers (Slave 0-3)
u8 mpu6500GetSlaveAddress(u8 num);
void mpu6500SetSlaveAddress(u8 num, u8 address);
u8 mpu6500GetSlaveRegister(u8 num);
void mpu6500SetSlaveRegister(u8 num, u8 reg);
bool mpu6500GetSlaveEnabled(u8 num);
void mpu6500SetSlaveEnabled(u8 num, bool enabled);
bool mpu6500GetSlaveWordByteSwap(u8 num);
void mpu6500SetSlaveWordByteSwap(u8 num, bool enabled);
bool mpu6500GetSlaveWriteMode(u8 num);
void mpu6500SetSlaveWriteMode(u8 num, bool mode);
bool mpu6500GetSlaveWordGroupOffset(u8 num);
void mpu6500setSlaveWordGroupOffset(u8 num, bool enabled);
u8 mpu6500GetSlaveDataLength(u8 num);
void mpu6500SetSlaveDataLength(u8 num, u8 length);

// I2C_SLV* registers (Slave 4)
u8 mpu6500GetSlave4Address(void);
void mpu6500SetSlave4Address(u8 address);
u8 mpu6500GetSlave4Register(void);
void mpu6500SetSlave4Register(u8 reg);
void mpu6500SetSlave4OutputByte(u8 data);
bool mpu6500GetSlave4Enabled(void);
void mpu6500SetSlave4Enabled(bool enabled);
bool mpu6500GetSlave4InterruptEnabled(void);
void mpu6500SetSlave4InterruptEnabled(bool enabled);
bool mpu6500GetSlave4WriteMode(void);
void mpu6500SetSlave4WriteMode(bool mode);
u8 mpu6500GetSlave4MasterDelay(void);
void mpu6500SetSlave4MasterDelay(u8 delay);
u8 mpu6500GetSlave4InputByte(void);

// I2C_MST_STATUS register
bool mpu6500GetPassthroughStatus(void);
bool mpu6500GetSlave4IsDone(void);
bool mpu6500GetLostArbitration(void);
bool mpu6500GetSlave4Nack(void);
bool mpu6500GetSlave3Nack(void);
bool mpu6500GetSlave2Nack(void);
bool mpu6500GetSlave1Nack(void);
bool mpu6500GetSlave0Nack(void);

// INT_PIN_CFG register
bool mpu6500GetInterruptMode(void);
void mpu6500SetInterruptMode(bool mode);
bool mpu6500GetInterruptDrive(void);
void mpu6500SetInterruptDrive(bool drive);
bool mpu6500GetInterruptLatch(void);
void mpu6500SetInterruptLatch(bool latch);
bool mpu6500GetInterruptLatchClear(void);
void mpu6500SetInterruptLatchClear(bool clear);
bool mpu6500GetFSyncInterruptLevel(void);
void mpu6500SetFSyncInterruptLevel(bool level);
bool mpu6500GetFSyncInterruptEnabled(void);
void mpu6500SetFSyncInterruptEnabled(bool enabled);
bool mpu6500GetI2CBypassEnabled(void);
void mpu6500SetI2CBypassEnabled(bool enabled);
bool mpu6500GetClockOutputEnabled(void);
void mpu6500SetClockOutputEnabled(bool enabled);

// INT_ENABLE register
u8 mpu6500GetIntEnabled(void);
void mpu6500SetIntEnabled(u8 enabled);
bool mpu6500GetIntFreefallEnabled(void);
void mpu6500SetIntFreefallEnabled(bool enabled);
bool mpu6500GetIntMotionEnabled(void);
void mpu6500SetIntMotionEnabled(bool enabled);
bool mpu6500GetIntZeroMotionEnabled(void);
void mpu6500SetIntZeroMotionEnabled(bool enabled);
bool mpu6500GetIntFIFOBufferOverflowEnabled(void);
void mpu6500SetIntFIFOBufferOverflowEnabled(bool enabled);
bool mpu6500GetIntI2CMasterEnabled(void);
void mpu6500SetIntI2CMasterEnabled(bool enabled);
bool mpu6500GetIntDataReadyEnabled(void);
void mpu6500SetIntDataReadyEnabled(bool enabled);

// INT_STATUS register
u8 mpu6500GetIntStatus(void);
bool mpu6500GetIntFreefallStatus(void);
bool mpu6500GetIntMotionStatus(void);
bool mpu6500GetIntZeroMotionStatus(void);
bool mpu6500GetIntFIFOBufferOverflowStatus(void);
bool mpu6500GetIntI2CMasterStatus(void);
bool mpu6500GetIntDataReadyStatus(void);

// ACCEL_*OUT_* registers
void mpu6500GetMotion9(s16* ax, s16* ay, s16* az, s16* gx, s16* gy, s16* gz, s16* mx, s16* my, s16* mz);
void mpu6500GetMotion6(s16* ax, s16* ay, s16* az, s16* gx, s16* gy, s16* gz);
void mpu6500GetAcceleration(s16* x, s16* y, s16* z);
s16 mpu6500GetAccelerationX(void);
s16 mpu6500GetAccelerationY(void);
s16 mpu6500GetAccelerationZ(void);

// TEMP_OUT_* registers
s16 mpu6500GetTemperature(void);

// GYRO_*OUT_* registers
void mpu6500GetRotation(s16* x, s16* y, s16* z);
s16 mpu6500GetRotationX(void);
s16 mpu6500GetRotationY(void);
s16 mpu6500GetRotationZ(void);

// EXT_SENS_DATA_* registers
u8 mpu6500GetExternalSensorByte(int position);
u16 mpu6500GetExternalSensorWord(int position);
uint32_t mpu6500GetExternalSensorDWord(int position);

// MOT_DETECT_STATUS register
bool mpu6500GetXNegMotionDetected(void);
bool mpu6500GetXPosMotionDetected(void);
bool mpu6500GetYNegMotionDetected(void);
bool mpu6500GetYPosMotionDetected(void);
bool mpu6500GetZNegMotionDetected(void);
bool mpu6500GetZPosMotionDetected(void);
bool mpu6500GetZeroMotionDetected(void);

// I2C_SLV*_DO register
void mpu6500SetSlaveOutputByte(u8 num, u8 data);

// I2C_MST_DELAY_CTRL register
bool mpu6500GetExternalShadowDelayEnabled(void);
void mpu6500SetExternalShadowDelayEnabled(bool enabled);
bool mpu6500GetSlaveDelayEnabled(u8 num);
void mpu6500SetSlaveDelayEnabled(u8 num, bool enabled);

// SIGNAL_PATH_RESET register
void rempu6500SetGyroscopePath(void);
void rempu6500SetAccelerometerPath(void);
void rempu6500SetTemperaturePath(void);

// MOT_DETECT_CTRL register
u8 mpu6500GetAccelerometerPowerOnDelay(void);
void mpu6500SetAccelerometerPowerOnDelay(u8 delay);
u8 mpu6500GetFreefallDetectionCounterDecrement(void);
void mpu6500SetFreefallDetectionCounterDecrement(u8 decrement);
u8 mpu6500GetMotionDetectionCounterDecrement(void);
void mpu6500SetMotionDetectionCounterDecrement(u8 decrement);

// USER_CTRL register
bool mpu6500GetFIFOEnabled(void);
void mpu6500SetFIFOEnabled(bool enabled);
bool mpu6500GetI2CMasterModeEnabled(void);
void mpu6500SetI2CMasterModeEnabled(bool enabled);
void mpu6500SwitchSPIEnabled(bool enabled);
void mpu6500ResetFIFO(void);
void mpu6500ResetI2CMaster(void);
void mpu6500ResetSensors(void);

// PWR_MGMT_1 register
void mpu6500Reset(void);
bool mpu6500GetSleepEnabled(void);
void mpu6500SetSleepEnabled(bool enabled);
bool mpu6500GetWakeCycleEnabled(void);
void mpu6500SetWakeCycleEnabled(bool enabled);
bool mpu6500GetTempSensorEnabled(void);
void mpu6500SetTempSensorEnabled(bool enabled);
u8 mpu6500GetClockSource(void);
void mpu6500SetClockSource(u8 source);

// PWR_MGMT_2 register
u8 mpu6500GetWakeFrequency(void);
void mpu6500SetWakeFrequency(u8 frequency);
bool mpu6500GetStandbyXAccelEnabled(void);
void mpu6500SetStandbyXAccelEnabled(bool enabled);
bool mpu6500GetStandbyYAccelEnabled(void);
void mpu6500SetStandbyYAccelEnabled(bool enabled);
bool mpu6500GetStandbyZAccelEnabled(void);
void mpu6500SetStandbyZAccelEnabled(bool enabled);
bool mpu6500GetStandbyXGyroEnabled(void);
void mpu6500SetStandbyXGyroEnabled(bool enabled);
bool mpu6500GetStandbyYGyroEnabled(void);
void mpu6500SetStandbyYGyroEnabled(bool enabled);
bool mpu6500GetStandbyZGyroEnabled(void);
void mpu6500SetStandbyZGyroEnabled(bool enabled);

// FIFO_COUNT_* registers
u16 mpu6500GetFIFOCount(void);

// FIFO_R_W register
u8 mpu6500GetFIFOByte(void);
void mpu6500SetFIFOByte(u8 data);
void mpu6500GetFIFOBytes(u8 *data, u8 length);

// WHO_AM_I register
u8 mpu6500GetDeviceID(void);
void mpu6500SetDeviceID(u8 id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
u8 mpu6500GetOTPBankValid(void);
void mpu6500SetOTPBankValid(bool enabled);
int8_t mpu6500GetXGyroOffset(void);
void mpu6500SetXGyroOffset(int8_t offset);

// YG_OFFS_TC register
int8_t mpu6500GetYGyroOffset(void);
void mpu6500SetYGyroOffset(int8_t offset);

// ZG_OFFS_TC register
int8_t mpu6500GetZGyroOffset(void);
void  mpu6500SetGyroOffset(int8_t offset);

// X_FINE_GAIN register
int8_t mpu6500GetXFineGain(void);
void mpu6500SetXFineGain(int8_t gain);

// Y_FINE_GAIN register
int8_t mpu6500GetYFineGain(void);
void mpu6500SetYFineGain(int8_t gain);

// Z_FINE_GAIN register
int8_t mpu6500GetZFineGain(void);
void mpu6500SetZFineGain(int8_t gain);

// XA_OFFS_* registers
s16 mpu6500GetXAccelOffset(void);
void mpu6500SetXAccelOffset(s16 offset);

// YA_OFFS_* register
s16 mpu6500GetYAccelOffset(void);
void mpu6500SetYAccelOffset(s16 offset);

// ZA_OFFS_* register
s16 mpu6500GetZAccelOffset(void);
void mpu6500SetZAccelOffset(s16 offset);

// XG_OFFS_USR* registers
s16 mpu6500GetXGyroOffsetUser(void);
void mpu6500SetXGyroOffsetUser(s16 offset);

// YG_OFFS_USR* register
s16 mpu6500GetYGyroOffsetUser(void);
void mpu6500SetYGyroOffsetUser(s16 offset);

// ZG_OFFS_USR* register
s16 mpu6500GetZGyroOffsetUser(void);
void mpu6500SetZGyroOffsetUser(s16 offset);

// INT_ENABLE register (DMP functions)
bool mpu6500GetIntPLLReadyEnabled(void);
void mpu6500SetIntPLLReadyEnabled(bool enabled);
bool mpu6500GetIntDMPEnabled(void);
void mpu6500SetIntDMPEnabled(bool enabled);

// DMP_INT_STATUS
bool mpu6500GetDMPInt5Status(void);
bool mpu6500GetDMPInt4Status(void);
bool mpu6500GetDMPInt3Status(void);
bool mpu6500GetDMPInt2Status(void);
bool mpu6500GetDMPInt1Status(void);
bool mpu6500GetDMPInt0Status(void);

// INT_STATUS register (DMP functions)
bool mpu6500GetIntPLLReadyStatus(void);
bool mpu6500GetIntDMPStatus(void);

// USER_CTRL register (DMP functions)
bool mpu6500GetDMPEnabled(void);
void mpu6500SetDMPEnabled(bool enabled);
void mpu6500ResetDMP(void);

// BANK_SEL register
void mpu6500SetMemoryBank(u8 bank, bool prefetchEnabled, bool userBank);

// MEM_START_ADDR register
void mpu6500SetMemoryStartAddress(u8 address);

// MEM_R_W register
u8 mpu6500ReadMemoryByte(void);
void mpu6500WriteMemoryByte(u8 data);
void mpu6500ReadMemoryBlock(u8 *data, u16 dataSize, u8 bank, u8 address);
bool mpu6500WriteMemoryBlock(const u8 *data, u16 dataSize, u8 bank, u8 address, bool verify);
bool mpu6500WriteProgMemoryBlock(const u8 *data, u16 dataSize, u8 bank, u8 address, bool verify);

bool mpu6500WriteDMPConfigurationSet(const u8 *data, u16 dataSize);
bool mpu6500WiteProgDMPConfigurationSet(const u8 *data, u16 dataSize);

// DMP_CFG_1 register
u8 mpu6500GetDMPConfig1(void);
void mpu6500SetDMPConfig1(u8 config);

// DMP_CFG_2 register
u8 mpu6500GetDMPConfig2(void);
void mpu6500SetDMPConfig2(u8 config);

#endif /* __MPU6500_H */

