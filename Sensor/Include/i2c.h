#ifndef __I2C_H 
#define __I2C_H 
// #include "stm32f4xx.h" 
// #include "stdbool.h" 

#define I2C_NO_INTERNAL_ADDRESS             (0xFFFF)
#define RCC_AHB1PERIPH_GPIOB                (0x00000002)
#define RCC_APB1PERIPH_I2C1                 (0x00200000)
#define GPIO_PIN_8                          (0x0100)/* Pin 8 selected */ 
#define GPIO_PIN_9                          (0x0200)/* Pin 9 selected */ 
#define GPIO_PINSOURCE8                     (0x08)
#define GPIO_PINSOURCE9                     (0x09)
#define GPIO_AF_I2C1                        (0x04)/* I2C1 Alternate Function mapping */ 
//传感器IIC总线速度 
#define I2C_SENSORS_CLOCK_SPEED             (400000)
#define I2C_DECK_CLOCK_SPEED                (400000)
#define I2C_NO_BLOCK                        (0)
#define I2C_SLAVE_ADDRESS7                  (0x30)
#define I2C_MAX_RETRIES                     (2)
#define I2C_MESSAGE_TIMEOUT                 (1000)
#define I2CDEV_LOOPS_PER_US                 (10)
#define I2CDEV_LOOPS_PER_MS                 (100000)
#define I2CDEV_CLK_TS                       (10*I2CDEV_LOOPS_PER_US)
#define GPIO_WAIT_FOR_HIGH(GPIO, PIN,TIME_OUT_CYCLES)\
{\ 
    int i=TIME_OUT_CYCLES;\ 
    while(((GPIO)->IDR&(PIN))==0&&i--);\ 
}
#define GPIO_WAIT_FOR_LOW(GPIO,PIN,TIME_OUT_CYCLES)\
{\ 
    int i=TIME_OUT_CYCLES;\
    while(((GPIO)->IDR&(PIN))!=0&&i--);\
}

#define I2C_ACK             (0U)
#define I2C_NACK            (1U)
#define I2C_WRITE           (0U)
#define I2C_READ            (1U)

/** 
* Structure used to capture the I2C message details. The structure is then 
* queued for processing by the I2C ISR. 
*/ 
struct I2C_Message
{
    uint32_t Message_Length;//< How many bytes of data to send or received. 
    uint8_t Slave_Address;//< The slave address of the device on the I2C bus. 
    uint8_t Retry_Count;//< The slave address of the device on the I2C bus. 
    uint8_t Direction;//< Direction of message 
    uint8_t Status;//< i2c status 
    bool Is_Internal_Address;//< Is internal address 16 bit. If false 8 bit. 
    uint16_t Internal_Address;//< Internal address of device. 
    uint8_t *Buffer;//< Pointer to the buffer from where data will be read for transmission, or into which received data will be placed. 
};

struct I2C_Def
{
    I2C_TypeDef* I2C_Port;
    uint32_t I2C_Perif;
    uint32_t I2C_EVIRQn;
    uint32_t I2C_ERIRQn;
    uint32_t I2C_Clock_Speed;
    uint32_t GPIO_SCL_Perif;
    GPIO_TypeDef* GPIO_SCL_Port;
    uint32_t GPIO_SCL_Pin;
    uint32_t GPIO_SCL_Pin_Source;
    uint32_t GPIO_SDA_Perif;
    GPIO_TypeDef* GPIO_SDA_Port;
    uint32_t GPIO_SDA_Pin;
    uint32_t GPIO_SDA_Pin_Source;
    uint32_t GPIO_AF;
};

struct I2C_Dev
{
    const struct I2cDef *Def;             //< Definition of the i2c 
    struct I2cMessage Tx_Message;         //< The I2C send message 
    uint32_t Message_Index;               //< Index of bytes sent/received 
    uint32_t Retry_Count;              //< Retries done 
};

extern struct I2C_Dev Sensor_Bus; 

void I2Cdev_Init(struct I2C_Dev* I2c); 

bool I2Cdev_Message_Transfer(struct I2C_Dev* I2c,struct I2cMessage* Message);

void I2Cdev_Create_Message_Int_Addr(struct I2cMessage *Message,
                                    uint8_t Slave_Address,
                                    bool Is_Internal_16,
                                    uint16_t Int_Address,
                                    uint8_t Direction,
                                    uint32_t Length,
                                    uint8_t *Buffer);

bool I2Cdev_Read(struct I2C_Dev *Dev, uint8_t Dev_Address, uint8_t Mem_Address, uint16_t Len, uint8_t *Data);
bool I2Cdev_Read16(struct I2C_Dev *Dev, uint8_t Dev_Address, uint16_t Mem_Address, uint16_t Len, uint8_t *Data);
int  I2Cdev_Init(struct I2C_Dev *Dev); 
bool I2Cdev_Read_Byte(struct I2C_Dev *Dev, uint8_t Dev_Address, uint8_t Mem_Address, uint8_t *Data);
bool I2Cdev_Read_Bit(struct I2C_Dev *Dev, uint8_t Dev_Address, uint8_t Mem_Address, uint8_t Bit_Num, uint8_t *Data); 
bool I2Cdev_Read_Bits(struct I2C_Dev *Dev, uint8_t Dev_Address, uint8_t Mem_Address, uint8_t Bit_Start, uint8_t Length, uint8_t *Data); 
bool I2Cdev_Write(struct I2C_Dev *Dev, uint8_t Dev_Address, uint8_t Mem_Address, uint16_t Len, uint8_t *Data); 
bool I2Cdev_Write16(struct I2C_Dev *Dev, uint8_t Dev_Address, uint16_t Mem_Address, uint16_t Len, uint8_t *Data); 
bool I2Cdev_Write_Byte(struct I2C_Dev *Dev, uint8_t Dev_Address, uint8_t Mem_Address, uint8_t Data); 
bool I2Cdev_Write_Bit(struct I2C_Dev *Dev, uint8_t Dev_Address, uint8_t Mem_Address, uint8_t Bit_Num, uint8_t Data); 
bool I2Cdev_Write_Bits(struct I2C_Dev *Dev, uint8_t Dev_Address, uint8_t Mem_Address, uint8_t Bit_Start, uint8_t Length, uint8_t Data); 

#endif //__I2CDEV_H