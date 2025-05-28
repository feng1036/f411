#ifndef __I2C_H
#define __I2C_H
#include "stm32f4xx.h" 
#include "stdbool.h"
#include "stdbool.h"


#define I2C_NO_INTERNAL_ADDRESS           0xFFFF
#define RCC_AHB1Periph_GPIOB            ((uint32_t)0x00000002)
#define RCC_APB1Periph_I2C1             ((uint32_t)0x00200000)
#define GPIO_Pin_8                      ((uint16_t)0x0100)  /* Pin 8 selected */
#define GPIO_Pin_9                         ((uint16_t)0x0200)  /* Pin 9 selected */
#define GPIO_PinSource8                    ((uint8_t)0x08)
#define GPIO_PinSource9                    ((uint8_t)0x09)
#define GPIO_AF_I2C1                      ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
//传感器IIC总线速度
#define I2C_SENSORS_CLOCK_SPEED    400000
#define I2C_DECK_CLOCK_SPEED    400000
#define I2C_NO_BLOCK            0
#define I2C_SLAVE_ADDRESS7      0x30
#define I2C_MAX_RETRIES         2
#define I2C_MESSAGE_TIMEOUT     (1000)
#define I2CDEV_LOOPS_PER_US      (10)
#define I2CDEV_LOOPS_PER_MS      (100000)    
#define I2CDEV_CLK_TS (10 * I2CDEV_LOOPS_PER_US)
#define GPIO_WAIT_FOR_HIGH(gpio, pin, timeoutcycles)\
{\
    int i = timeoutcycles;\
    while(((gpio->IDR & pin) == 0) && i--);\
}
#define GPIO_WAIT_FOR_LOW(gpio, pin, timeoutcycles) \
{\
    int i = timeoutcycles;\
    while(((gpio->IDR & pin) != 0) && i--);\
}

typedef enum
{
    i2cAck,
    i2cNack
} I2cStatus;

typedef enum
{
    i2cWrite,
    i2cRead
} I2cDirection;

/**
 * Structure used to capture the I2C message details.  The structure is then
 * queued for processing by the I2C ISR.
 */
typedef struct _I2cMessage
{
    uint32_t         messageLength;          //< How many bytes of data to send or received.
    uint8_t          slaveAddress;          //< The slave address of the device on the I2C bus.
    uint8_t          nbrOfRetries;      //< The slave address of the device on the I2C bus.
    I2cDirection     direction;         //< Direction of message
    I2cStatus        status;            //< i2c status
    //xQueueHandle     clientQueue;       //< Queue to send received messages to.
    bool             isInternal16bit;   //< Is internal address 16 bit. If false 8 bit.
    uint16_t         internalAddress;   //< Internal address of device.
    uint8_t          *buffer;           //< Pointer to the buffer from where data will be read for transmission, or into which received data will be placed.
} I2cMessage;

typedef struct
{
    I2C_TypeDef*        i2cPort;
    uint32_t            i2cPerif;
    uint32_t            i2cEVIRQn;
    uint32_t            i2cERIRQn;
    uint32_t            i2cClockSpeed;
    uint32_t            gpioSCLPerif;
    GPIO_TypeDef*       gpioSCLPort;
    uint32_t            gpioSCLPin;
    uint32_t            gpioSCLPinSource;
    uint32_t            gpioSDAPerif;
    GPIO_TypeDef*       gpioSDAPort;
    uint32_t            gpioSDAPin;
    uint32_t            gpioSDAPinSource;
    uint32_t            gpioAF;
} I2cDef;

typedef struct
{
    const I2cDef *def;                    //< Definition of the i2c
    I2cMessage txMessage;                 //< The I2C send message
    uint32_t messageIndex;                //< Index of bytes sent/received
    uint32_t nbrOfretries;                //< Retries done
    // SemaphoreHandle_t isBusFreeSemaphore; //< Semaphore to block during transaction.
    // SemaphoreHandle_t isBusFreeMutex;     //< Mutex to protect buss
} I2cDrv;

// Definitions of i2c busses found in c file.
extern I2cDrv sensorsBus;
/**
 * Initialize i2c peripheral as defined by static I2cDef structs.
 */
void i2cdrvInit(I2cDrv* i2c);

/**
 * Send or receive a message over the I2C bus.
 *
 * The message is synchrony by semapthore and uses interrupts to transfer the message.
 *
 * @param i2c      i2c bus to use.
 * @param message     An I2cMessage struct containing all the i2c message
 *                 Information. Message status will be altered if nack.
 * @return         true if successful, false otherwise.
 */
bool i2cdrvMessageTransfer(I2cDrv* i2c, I2cMessage* message);


/**
 * Create a message to transfer with internal "reg" address. Will first do a write
 * of one or two bytes depending of IsInternal16 and then write/read the data.
 *
 * @param message       pointer to message struct that will be filled in.
 * @param slaveAddress  i2c slave address
 * @param IsInternal16  It true 16bit reg address else 8bit.
 * @param direction     i2cWrite or i2cRead
 * @param length        Length of message
 * @param buffer        pointer to buffer of send/receive data
 */
void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                             uint8_t  slaveAddress,
                             bool IsInternal16,
                             uint16_t intAddress,
                             I2cDirection  direction,
                             uint32_t length,
                             uint8_t  *buffer);

#endif
