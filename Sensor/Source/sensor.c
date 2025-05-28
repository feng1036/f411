#include "sensor.h"
#include "i2c.h"
#include "stdbool.h"

static uint8_t MPU_Addr;
static uint8_t BMP_Addr;
static struct I2C_Dev* MPU_Device;
static struct I2C_Dev* BMP_Device;
static bool MPU_Is_Init=false;
static bool BMP_Is_Init=false;

#define MPU6500_DEFAULT_ADDRESS            (0x69)

// Register Map
#define MPU6500_REG_I2C_MST_CTRL           (0x24)
#define MPU6500_REG_I2C_SLV0_ADDR          (0x25)
#define MPU6500_REG_I2C_SLV0_REG           (0x26)
#define MPU6500_REG_I2C_SLV0_CTRL          (0x27)
#define MPU6500_REG_I2C_SLV4_ADDR          (0x31)
#define MPU6500_REG_I2C_SLV4_CTRL          (0x34)
#define MPU6500_REG_INT_PIN_CFG            (0x37)
#define MPU6500_REG_INT_ENABLE             (0x38)
#define MPU6500_REG_ACCEL_XOUT_H           (0x3B)
#define MPU6500_REG_I2C_MST_DELAY_CTRL     (0x67)
#define MPU6500_REG_USER_CTRL              (0x6A)
#define MPU6500_REG_PWR_MGMT_1             (0x6B)

// Bit Definitions
#define MPU6500_BIT_CFG_DLPF_CFG           (2)
#define MPU6500_BIT_GCONFIG_FS_SEL         (4)
#define MPU6500_BIT_ACONFIG_AFS_SEL        (4)
#define MPU6500_BIT_ACONFIG2_DLPF          (0)
#define MPU6500_BIT_WAIT_FOR_ES            (6)
#define MPU6500_BIT_I2C_MST_P_NSR          (4)
#define MPU6500_BIT_I2C_MST_CLK            (3)
#define MPU6500_BIT_I2C_SLV_EN             (7)
#define MPU6500_BIT_I2C_SLV_LEN            (3)
#define MPU6500_BIT_I2C_SLV4_MST_DLY       (4)
#define MPU6500_BIT_INTCFG_INT_LEVEL       (7)
#define MPU6500_BIT_INTCFG_INT_OPEN        (6)
#define MPU6500_BIT_INTCFG_LATCH_INT_EN    (5)
#define MPU6500_BIT_INTCFG_INT_RD_CLEAR    (4)
#define MPU6500_BIT_INTCFG_I2C_BYPASS_EN   (1)
#define MPU6500_BIT_INTERRUPT_DATA_RDY     (0)
#define MPU6500_BIT_USERCTRL_I2C_MST_EN    (5)
#define MPU6500_BIT_PWR1_DEVICE_RESET      (7)
#define MPU6500_BIT_PWR1_SLEEP             (6)
#define MPU6500_BIT_PWR1_TEMP_DIS          (3)
#define MPU6500_BIT_PWR1_CLKSEL            (2)

// Length Definitions
#define MPU6500_LEN_CFG_DLPF_CFG           (3)
#define MPU6500_LEN_GCONFIG_FS_SEL         (2)
#define MPU6500_LEN_ACONFIG_AFS_SEL        (2)
#define MPU6500_LEN_ACONFIG2_DLPF          (2)
#define MPU6500_LEN_I2C_MST_CLK            (4)
#define MPU6500_LEN_I2C_SLV_LEN            (4)
#define MPU6500_LEN_I2C_SLV4_MST_DLY       (5)
#define MPU6500_LEN_PWR1_CLKSEL            (3)

// Configuration Values
#define MPU6500_DLPF_BW_98                 (0x02)
#define MPU6500_GYRO_FS_2000               (0x03)
#define MPU6500_ACCEL_DLPF_BW_41           (0x03)
#define MPU6500_ACCEL_FS_16                (0x03)
#define MPU6500_CLOCK_PLL_XGYRO            (0x01)
void MPU6500_Init(struct I2C_Dev* I2c_Port)
{
    if(MPU_Is_Init) 
        return;
    MPU_Device=I2c_Port;
    MPU_Addr=MPU6500_DEFAULT_ADDRESS;
    MPU_Is_Init=true;
    RMP_Thd_Delay(100);
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x6B,7,true);
    RMP_Thd_Delay(200); // 延时等待寄存器复位     
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x6B,6,false);// 唤醒MPU6500      
    RMP_Thd_Delay(100);
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x6B,2,3,0x01); // 设置X轴陀螺作为时钟
    RMP_Thd_Delay(100);
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x6B,3,false);// 使能温度传感器  
    I2Cdev_Write_Byte(MPU_Device,MPU_Addr,0x38,false);// 关闭中断  
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x37,1,true);// 旁路模式，磁力计和气压连接到主IIC
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x1B,4,2,3);// 设置陀螺量程
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x1C,4,2,3);// 设置加速计量程 
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x1D,0,2,3);// 设置加速计数字低通滤波
    I2Cdev_Write_Byte(MPU_Device,MPU_Addr,0x19,0);//设置采样速率: 1000 / (1 + 0) = 1000Hz
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x1A,2,3,2);// 设置陀螺数字低通滤波
}

static bool BMP_Is_Init=false;

static struct BMP280_Calib_Data Calib_Data;

bool BMP280_Init(struct I2C_Dev* I2c_Port)
{
    uint8_t Chip_Id;
    if(BMP_Is_Init) return true;
    BMP_Device=I2c_Port;
    BMP_Addr=BMP280_I2C_ADDRESS;
    if(!I2Cdev_Read_Byte(BMP_Device,BMP_Addr,0xD0,&Chip_Id))
        return false;
    if(Chip_Id!=BMP280_DEFAULT_CHIP_ID)
        return false;
    if(!I2Cdev_Read(BMP_Device,BMP_Addr,0x88,24,(uint8_t*)&Calib_Data))
        return false;
    uint8_t Ctrl_Meas=0x93;
    I2Cdev_Write_Byte(BMP_Device,BMP_Addr,0xF4,Ctrl_Meas);
    I2Cdev_Write_Byte(BMP_Device,BMP_Addr,0xF5,0x14);
    BMP_Is_Init=true;
    return true;
}
//温度补偿公式
uint32_t BMP280_Compensate_T(int32_t Adc_T)
{
    int32_t Var1,Var2;
    Var1=(((Adc_T>>3)-((int32_t)Calib_Data.Dig_T1<<1))*((int32_t)Calib_Data.Dig_T2))>>11;
    Var2=(((((Adc_T>>4)-((int32_t)Calib_Data.Dig_T1))*((Adc_T>>4)-((int32_t)Calib_Data.Dig_T1)))>>12)*((int32_t)Calib_Data.Dig_T3))>>14;
    Calib_Data.T_Fine=Var1+Var2;
    return (uint32_t)((Calib_Data.T_Fine*5+128)>>8);
}
//气压补偿公式
uint32_t BMP280_Compensate_P(int32_t Adc_P)
{
    int64_t Var1,Var2,P;
    Var1=((int64_t)Calib_Data.T_Fine)-128000;
    Var2=Var1*Var1*(int64_t)Calib_Data.Dig_P6;
    Var2=Var2+((Var1*(int64_t)Calib_Data.Dig_P5)<<17);
    Var2=Var2+(((int64_t)Calib_Data.Dig_P4)<<35);
    Var1=((Var1*Var1*(int64_t)Calib_Data.Dig_P3)>>8)+((Var1*(int64_t)Calib_Data.Dig_P2)<<12);
    Var1=((((int64_t)1)<<47)+Var1)*((int64_t)Calib_Data.Dig_P1)>>33;
    if(Var1==0) return 0;
    P=1048576-Adc_P;
    P=(((P<<31)-Var2)*3125)/Var1;
    Var1=(((int64_t)Calib_Data.Dig_P9)*(P>>13)*(P>>13))>>25;
    Var2=(((int64_t)Calib_Data.Dig_P8)*P)>>19;
    P=((P+Var1+Var2)>>8)+(((int64_t)Calib_Data.Dig_P7)<<4);
    return (uint32_t)P;
}
//海拔计算公式
float BMP280_Pressure_To_Altitude(float* Pressure)
{
    if(*Pressure>0)
    {
        return 44330.0f*(powf((101325.0f/(*Pressure)),0.190295f)-1.0f);
    }
    return 0.0f;
}

/*设置传感器从模式读取*/
void Sensor_Setup_Slave_Read(void)
{
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x34,4,5,19);// 从机读取速率: 100Hz = (1000Hz / (1 + 9))
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x37,1,false);//主机模式
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x24,6,true);
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x37,7,0);//中断高电平有效
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x37,6,0);//推挽输出
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x37,5,0);//中断锁存模式
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x37,4,1);//中断清除模式
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x24,4,false);//关闭从机读写传输
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x24,3,4,13);// 设置i2c速度400kHz
    // 设置MPU6500主机要读取BMP280的寄存器
    I2Cdev_Write_Byte(MPU_Device,MPU_Addr,0x28,0xF6);// 设置气压计状态寄存器为1号从机
    I2Cdev_Write_Byte(MPU_Device,MPU_Addr,0x29,0xF3);// 从机1需要读取的寄存器
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x2A,3,4,1);// 读取1个字节
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x2A,7,true);
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x2A,7,true);
    I2Cdev_Write_Byte(MPU_Device,MPU_Addr,0x2B,0xF6);// 设置气压计数据寄存器为2号从机
    I2Cdev_Write_Byte(MPU_Device,MPU_Addr,0x2C,0xF7);// 从机2需要读取的寄存器
    I2Cdev_Write_Bits(MPU_Device,MPU_Addr,0x2D,3,4,6);// 读取6个字节
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x2D,7,true);
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x2D,7,true);
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x6A,5,true);//使能mpu6500主机模式
    I2Cdev_Write_Bit(MPU_Device,MPU_Addr,0x38,0,true);//数据就绪中断使能
}
