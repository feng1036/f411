#include <string.h>
#include "stm32f4xx.h" 
#include "i2c_drv.h"
#include "config.h"
#include "delay.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//传感器IIC总线速度
#define I2C_SENSORS_CLOCK_SPEED	400000
#define I2C_DECK_CLOCK_SPEED	400000

// Misc constants.
#define I2C_NO_BLOCK			0
#define I2C_SLAVE_ADDRESS7      0x30
#define I2C_MAX_RETRIES         2
#define I2C_MESSAGE_TIMEOUT     (1000)

// Delay is approx 0.01us per loop @96Mhz
#define I2CDEV_LOOPS_PER_US  	(10)
#define I2CDEV_LOOPS_PER_MS  	(100000)	
#define I2CDEV_CLK_TS (10 * I2CDEV_LOOPS_PER_US)

#define GPIO_WAIT_FOR_HIGH(gpio, pin, timeoutcycles)\
{\
	int i = timeoutcycles;\
	while(GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET && i--);\
}

#define GPIO_WAIT_FOR_LOW(gpio, pin, timeoutcycles) \
{\
	int i = timeoutcycles;\
	while(GPIO_ReadInputDataBit(gpio, pin) == Bit_SET && i--);\
}

  
/**
 * IIC底层驱动初始化
 */
static void i2cdrvInitBus(I2cDrv* i2c);
/**
 * 重启IIC总线
 */
static void i2cdrvTryToRestartBus(I2cDrv* i2c);
/**
 * 环路延时
 */
static inline void i2cdrvRoughLoopDelay(uint32_t us);
/**
 * 解锁IIC总线
 */
static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA);

static const I2cDef sensorsBusDef =
{
	.i2cPort            = I2C1,
	.i2cPerif           = RCC_APB1Periph_I2C1,
	.i2cEVIRQn          = I2C1_EV_IRQn,
	.i2cERIRQn          = I2C1_ER_IRQn,
	.i2cClockSpeed      = I2C_SENSORS_CLOCK_SPEED,
	.gpioSCLPerif       = RCC_AHB1Periph_GPIOB,
	.gpioSCLPort        = GPIOB,
	.gpioSCLPin         = GPIO_Pin_8,
	.gpioSCLPinSource   = GPIO_PinSource8,
	.gpioSDAPerif       = RCC_AHB1Periph_GPIOB,
	.gpioSDAPort        = GPIOB,
	.gpioSDAPin         = GPIO_Pin_9,
	.gpioSDAPinSource   = GPIO_PinSource9,
	.gpioAF             = GPIO_AF_I2C1,
};

/**
 * 传感器总线
 */
I2cDrv sensorsBus =
{
	.def                = &sensorsBusDef,
};

static const I2cDef deckBusDef =
{
	.i2cPort            = I2C3,
	.i2cPerif           = RCC_APB1Periph_I2C3,
	.i2cEVIRQn          = I2C3_EV_IRQn,
	.i2cERIRQn          = I2C3_ER_IRQn,
	.i2cClockSpeed      = I2C_DECK_CLOCK_SPEED,
	.gpioSCLPerif       = RCC_AHB1Periph_GPIOA,
	.gpioSCLPort        = GPIOA,
	.gpioSCLPin         = GPIO_Pin_8,
	.gpioSCLPinSource   = GPIO_PinSource8,
	.gpioSDAPerif       = RCC_AHB1Periph_GPIOB,
	.gpioSDAPort        = GPIOB,
	.gpioSDAPin         = GPIO_Pin_4,
	.gpioSDAPinSource   = GPIO_PinSource4,
	.gpioAF             = GPIO_AF_I2C3,
};

I2cDrv deckBus =
{
	.def                = &deckBusDef,
};


/**
 * 环路延时
 */
static inline void i2cdrvRoughLoopDelay(uint32_t us)
{
	volatile uint32_t delay = 0;
//	delay_us(us);
	for(delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay) { };
}
/**
 * 重启IIC总线
 */
static void i2cdrvTryToRestartBus(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}

/**
 * IIC底层驱动初始化
 */
static void i2cdrvInitBus(I2cDrv* i2c)
{
	I2C_InitTypeDef  I2C_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIOA clock
	RCC_AHB1PeriphClockCmd(i2c->def->gpioSDAPerif, ENABLE);
    //RCC->AHB1ENR |= i2c->def->gpioSDAPerif;
	RCC_AHB1PeriphClockCmd(i2c->def->gpioSCLPerif, ENABLE);
    //RCC->AHB1ENR |= i2c->def->gpioSCLPerif;
	// Enable I2C_SENSORS clock
	RCC_APB1PeriphClockCmd(i2c->def->i2cPerif, ENABLE);	
	
	// Configure I2C_SENSORS pins to unlock bus.
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; // SCL
	GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; // SDA
	GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

	i2cdrvdevUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort, i2c->def->gpioSCLPin, i2c->def->gpioSDAPin);

	// Configure I2C_SENSORS pins for AF.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; // SCL
	GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; // SDA
	GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

	//Map gpios to alternate functions
	GPIO_PinAFConfig(i2c->def->gpioSCLPort, i2c->def->gpioSCLPinSource, i2c->def->gpioAF);
	GPIO_PinAFConfig(i2c->def->gpioSDAPort, i2c->def->gpioSDAPinSource, i2c->def->gpioAF);

	// I2C_SENSORS configuration
	I2C_DeInit(i2c->def->i2cPort);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = i2c->def->i2cClockSpeed;
	I2C_Init(i2c->def->i2cPort, &I2C_InitStructure);

	// Enable I2C_SENSORS error interrupts
	I2C_ITConfig(i2c->def->i2cPort, I2C_IT_ERR, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cEVIRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cERIRQn;
	NVIC_Init(&NVIC_InitStructure);

	i2c->isBusFreeSemaphore = xSemaphoreCreateBinary();
	i2c->isBusFreeMutex = xSemaphoreCreateMutex();
}
/**
 * 解锁IIC总线
 */
static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA)
{
	GPIO_SetBits(portSDA, pinSDA);
	/* Check SDA line to determine if slave is asserting bus and clock out if so */
	while(GPIO_ReadInputDataBit(portSDA, pinSDA) == Bit_RESET)
	{
		/* Set clock high */
		GPIO_SetBits(portSCL, pinSCL);
		/* Wait for any clock stretching to finish. */
		GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, I2CDEV_LOOPS_PER_MS);
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

		/* Generate a clock cycle */
		GPIO_ResetBits(portSCL, pinSCL);
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
		GPIO_SetBits(portSCL, pinSCL);
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	}

	/* Generate a start then stop condition */
	GPIO_SetBits(portSCL, pinSCL);
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	GPIO_ResetBits(portSDA, pinSDA);
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	GPIO_ResetBits(portSDA, pinSDA);
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

	/* Set data and clock high and wait for any clock stretching to finish. */
	GPIO_SetBits(portSDA, pinSDA);
	GPIO_SetBits(portSCL, pinSCL);
	GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, I2CDEV_LOOPS_PER_MS);
	/* Wait for data to be high */
	GPIO_WAIT_FOR_HIGH(portSDA, pinSDA, I2CDEV_LOOPS_PER_MS);
}


/**
 * IIC外设初始化
 */
void i2cdrvInit(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}


/**
 * 创建一个用于传输内部寄存器的信息
 */
void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                             uint8_t  slaveAddress,
                             bool IsInternal16,
                             uint16_t intAddress,
                             I2cDirection  direction,	
                             uint32_t length,
                             uint8_t  *buffer)
{
	message->slaveAddress = slaveAddress;
	message->direction = direction;
	message->isInternal16bit = IsInternal16;
	message->internalAddress = intAddress;
	message->messageLength = length;
	message->status = i2cAck;
	message->buffer = buffer;
	message->nbrOfRetries = I2C_MAX_RETRIES;
}

/**
 * I2C轮询方式写数据
 */
static bool i2cWritePoll(I2cDrv* i2c, I2cMessage* message)
{
    uint16_t timeout;
    uint16_t i;

    // 发送起始位
    i2c->def->i2cPort->CR1 |= I2C_CR1_START;
    
    // 等待起始位发送完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(i2c->def->i2cPort->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0) return false;
    }
    
    // 发送设备地址(写)
    i2c->def->i2cPort->DR = message->slaveAddress << 1;
    
    // 等待地址发送完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(i2c->def->i2cPort->SR1 & I2C_SR1_ADDR)) {
        if (--timeout == 0) return false;
        // 检查是否有ACK错误
        if (i2c->def->i2cPort->SR1 & I2C_SR1_AF) {
            i2c->def->i2cPort->SR1 &= ~I2C_SR1_AF; // 清除AF标志
            i2c->def->i2cPort->CR1 |= I2C_CR1_STOP; // 发送停止位
            return false;
        }
    }
    
    // 清除ADDR标志
    uint16_t temp = i2c->def->i2cPort->SR2;
    (void)temp;
    
    // 如果有内部地址，先发送内部地址
    if (message->internalAddress != I2C_NO_INTERNAL_ADDRESS) {
        if (message->isInternal16bit) {
            // 发送内部地址高字节
            i2c->def->i2cPort->DR = (message->internalAddress & 0xFF00) >> 8;
            timeout = I2C_MESSAGE_TIMEOUT;
            while (!(i2c->def->i2cPort->SR1 & I2C_SR1_TXE)) {
                if (--timeout == 0) return false;
            }
        }
        
        // 发送内部地址低字节
        i2c->def->i2cPort->DR = message->internalAddress & 0x00FF;
        timeout = I2C_MESSAGE_TIMEOUT;
        while (!(i2c->def->i2cPort->SR1 & I2C_SR1_TXE)) {
            if (--timeout == 0) return false;
        }
        
        // 如果是读操作，需要发送重复起始位
        if (message->direction == i2cRead) {
            return true; // 返回成功，继续发送重复起始位
        }
    }
    
    // 发送数据(只在写操作时)
    if (message->direction == i2cWrite) {
        for (i = 0; i < message->messageLength; i++) {
            i2c->def->i2cPort->DR = message->buffer[i];
            
            timeout = I2C_MESSAGE_TIMEOUT;
            while (!(i2c->def->i2cPort->SR1 & I2C_SR1_TXE)) {
                if (--timeout == 0) return false;
            }
        }
    }
    
    // 等待传输完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(i2c->def->i2cPort->SR1 & I2C_SR1_BTF)) {
        if (--timeout == 0) return false;
    }
    
    // 发送停止位
    i2c->def->i2cPort->CR1 |= I2C_CR1_STOP;
    
    return true;
}

/**
 * I2C轮询方式读数据
 */
static bool i2cReadPoll(I2cDrv* i2c, I2cMessage* message)
{
    uint16_t timeout;
    uint16_t i;
    
    // 发送起始位
    i2c->def->i2cPort->CR1 |= I2C_CR1_START;
    
    // 等待起始位发送完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(i2c->def->i2cPort->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0) return false;
    }
    
    // 发送设备地址(读)
    i2c->def->i2cPort->DR = (message->slaveAddress << 1) | 0x01;
    
    // 等待地址发送完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(i2c->def->i2cPort->SR1 & I2C_SR1_ADDR)) {
        if (--timeout == 0) return false;
        // 检查是否有ACK错误
        if (i2c->def->i2cPort->SR1 & I2C_SR1_AF) {
            i2c->def->i2cPort->SR1 &= ~I2C_SR1_AF; // 清除AF标志
            i2c->def->i2cPort->CR1 |= I2C_CR1_STOP; // 发送停止位
            return false;
        }
    }
    
    // 配置ACK
    if (message->messageLength == 1) {
        // 单字节读取，需要在清除ADDR前禁用ACK
        i2c->def->i2cPort->CR1 &= ~I2C_CR1_ACK;
    } else {
        // 多字节读取，启用ACK
        i2c->def->i2cPort->CR1 |= I2C_CR1_ACK;
    }
    
    // 清除ADDR标志
    uint16_t temp = i2c->def->i2cPort->SR2;
    (void)temp;
    
    // 读取数据
    for (i = 0; i < message->messageLength; i++) {
        if (i == message->messageLength - 1) {
            // 最后一个字节发送NACK
            i2c->def->i2cPort->CR1 &= ~I2C_CR1_ACK;
        }
        
        if (i == message->messageLength - 1) {
            // 最后一个字节之前发送停止位
            i2c->def->i2cPort->CR1 |= I2C_CR1_STOP;
        }
        
        // 等待接收数据
        timeout = I2C_MESSAGE_TIMEOUT;
        while (!(i2c->def->i2cPort->SR1 & I2C_SR1_RXNE)) {
            if (--timeout == 0) return false;
        }
        
        // 读取数据
        message->buffer[i] = i2c->def->i2cPort->DR;
    }
    
    return true;
}

/**
 * 发送或者接收IIC信息(轮询方式)
 */
bool i2cdrvMessageTransfer(I2cDrv* i2c, I2cMessage* message)
{
    bool status = false;
    int retries = message->nbrOfRetries;

    xSemaphoreTake(i2c->isBusFreeMutex, portMAX_DELAY); // 保护消息数据

    while (retries >= 0) {
        // 写操作或需要写内部地址
        if (message->direction == i2cWrite || 
            message->internalAddress != I2C_NO_INTERNAL_ADDRESS) {
            status = i2cWritePoll(i2c, message);
            
            // 如果成功且是读操作，需要发送重复起始位进行读取
            if (status && message->direction == i2cRead) {
                status = i2cReadPoll(i2c, message);
            }
        } else {
            // 直接读操作
            status = i2cReadPoll(i2c, message);
        }
        
        if (status) {
            // 传输成功
            break;
        }
        
        retries--;
        i2cdrvRoughLoopDelay(100); // 延时一段时间再重试
    }
    
    if (!status) {
        // 所有重试失败，尝试重启总线
        i2cdrvTryToRestartBus(i2c);
    }
    
    xSemaphoreGive(i2c->isBusFreeMutex);
    return status;
}
