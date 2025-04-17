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
 * IIC DMA初始化
 */
static void i2cdrvDmaSetupBus(I2cDrv* i2c);
/**
 * 启动IIC传输
 */
static void i2cdrvStartTransfer(I2cDrv *i2c);
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
/**
 * 清除DMA数据流
 */
static void i2cdrvClearDMA(I2cDrv* i2c);
/**
 * 事件中断服务函数
 */
static void i2cdrvEventIsrHandler(I2cDrv* i2c);
/**
 * 错误中断服务函数
 */
static void i2cdrvErrorIsrHandler(I2cDrv* i2c);
/**
 * DMA中断服务函数
 */
static void i2cdrvDmaIsrHandler(I2cDrv* i2c);

/**
 * 传感器总线定义
 */
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
	.dmaPerif           = RCC_AHB1Periph_DMA1,
	.dmaChannel         = DMA_Channel_1,
	.dmaRxStream        = DMA1_Stream0,
	.dmaRxIRQ           = DMA1_Stream0_IRQn,
	.dmaRxTCFlag        = DMA_FLAG_TCIF0,
	.dmaRxTEFlag        = DMA_FLAG_TEIF0,
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
	.dmaPerif           = RCC_AHB1Periph_DMA1,
	.dmaChannel         = DMA_Channel_3,
	.dmaRxStream        = DMA1_Stream2,
	.dmaRxIRQ           = DMA1_Stream2_IRQn,
	.dmaRxTCFlag        = DMA_FLAG_TCIF2,
	.dmaRxTEFlag        = DMA_FLAG_TEIF2,
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
 * 启动IIC传输
 */
static void i2cdrvStartTransfer(I2cDrv *i2c)
{
	if (i2c->txMessage.direction == i2cRead)
	{
		//改为直接操纵寄存器进行传输
		
	}

	I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
	I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT, ENABLE);
	i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
}

static void i2cTryNextMessage(I2cDrv* i2c)
{
	i2c->def->i2cPort->CR1 = (I2C_CR1_STOP | I2C_CR1_PE);
	I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
}

static void i2cNotifyClient(I2cDrv* i2c)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(i2c->isBusFreeSemaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
 * 重启IIC总线
 */
static void i2cdrvTryToRestartBus(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}
/**
 * IIC DMA初始化
 */
static void i2cdrvDmaSetupBus(I2cDrv* i2c)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(i2c->def->dmaPerif, ENABLE);

	// RX DMA Channel Config
	i2c->DMAStruct.DMA_Channel = i2c->def->dmaChannel;
	i2c->DMAStruct.DMA_PeripheralBaseAddr = (uint32_t)&i2c->def->i2cPort->DR;
	i2c->DMAStruct.DMA_Memory0BaseAddr = 0;
	i2c->DMAStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	i2c->DMAStruct.DMA_BufferSize = 0;
	i2c->DMAStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	i2c->DMAStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	i2c->DMAStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	i2c->DMAStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	i2c->DMAStruct.DMA_Mode = DMA_Mode_Normal;
	i2c->DMAStruct.DMA_Priority = DMA_Priority_High;
	i2c->DMAStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	i2c->DMAStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	i2c->DMAStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	i2c->DMAStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	NVIC_InitStructure.NVIC_IRQChannel = i2c->def->dmaRxIRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
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
	RCC_AHB1PeriphClockCmd(i2c->def->gpioSCLPerif, ENABLE);
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

	i2cdrvDmaSetupBus(i2c);

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

//-----------------------------------------------------------
/**
 * IIC外设初始化
 */
void i2cdrvInit(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}

/**
 * 创建一个传输信息
 */
void i2cdrvCreateMessage(I2cMessage *message,
                      uint8_t  slaveAddress,
                      I2cDirection  direction,	
                      uint32_t length,
                      uint8_t  *buffer)
{
	message->slaveAddress = slaveAddress;
	message->direction = direction;
	message->isInternal16bit = false;
	message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
	message->messageLength = length;
	message->status = i2cAck;
	message->buffer = buffer;
	message->nbrOfRetries = I2C_MAX_RETRIES;
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

