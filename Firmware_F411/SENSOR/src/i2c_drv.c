#include <string.h>
#include "stm32f4xx.h" 
#include "i2c_drv.h"
#include "delay.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

 // IIC底层驱动初始化
static void i2cdrvInitBus(I2cDrv* i2c);

//重启IIC总线
static void i2cdrvTryToRestartBus(I2cDrv* i2c);

//环路延时
static inline void i2cdrvRoughLoopDelay(uint32_t us);

//解锁IIC总线
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

//传感器总线
I2cDrv sensorsBus =
{
	.def                = &sensorsBusDef,
};

//环路延时
static inline void i2cdrvRoughLoopDelay(uint32_t us)
{
	volatile uint32_t delay = 0;
	for(delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay) { };
}

//重启IIC总线
static void i2cdrvTryToRestartBus(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}

//IIC底层驱动初始化
static void i2cdrvInitBus(I2cDrv* i2c)
{
    RCC->AHB1ENR |= ((uint32_t)0x00000002);
    
    RCC->AHB1ENR |= ((uint32_t)0x00000002);
    
    RCC->APB1ENR |= ((uint32_t)0x00200000);

    GPIOB->MODER  &= ~(GPIO_MODER_MODER0 << (8 * 2));
    GPIOB->MODER |= (((uint32_t)(0x01)) << (8 * 2));
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (8 * 2));
    GPIOB->OSPEEDR |= ((uint32_t)(0x02) << (8 * 2));
    /* Output mode configuration*/
    GPIOB->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)(8)));
    GPIOB->OTYPER |= (((uint16_t)(0x01)) << ((uint16_t)(8)));
    /* Pull-up Pull down resistor configuration*/
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)(8) * 2));
    GPIOB->PUPDR |= (((uint32_t)(0x00)) << (8 * 2));

    GPIOB->MODER  &= ~(GPIO_MODER_MODER0 << (9 * 2));
    GPIOB->MODER |= (((uint32_t)(0x01)) << (9 * 2));
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (9 * 2));
    GPIOB->OSPEEDR |= ((uint32_t)(0x02) << (9 * 2));
    /* Output mode configuration*/
    GPIOB->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)(9)));
    GPIOB->OTYPER |= (((uint16_t)(0x01)) << ((uint16_t)(9)));
    /* Pull-up Pull down resistor configuration*/
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)(9) * 2));
    GPIOB->PUPDR |= (((uint32_t)(0x00)) << (9 * 2));

	i2cdrvdevUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort, i2c->def->gpioSCLPin, i2c->def->gpioSDAPin);

    GPIOB->MODER  &= ~(GPIO_MODER_MODER0 << (8 * 2));
    GPIOB->MODER |= (((uint32_t)(0x02)) << (8 * 2));
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (8 * 2));
    GPIOB->OSPEEDR |= ((uint32_t)(0x02) << (8 * 2));
    /* Output mode configuration*/
    GPIOB->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)(8)));
    GPIOB->OTYPER |= (((uint16_t)(0x01)) << ((uint16_t)(8)));
    /* Pull-up Pull down resistor configuration*/
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)(8) * 2));
    GPIOB->PUPDR |= (((uint32_t)(0x00)) << (8 * 2));

    GPIOB->MODER  &= ~(GPIO_MODER_MODER0 << (9 * 2));
    GPIOB->MODER |= (((uint32_t)(0x02)) << (9 * 2));
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (9 * 2));
    GPIOB->OSPEEDR |= ((uint32_t)(0x02) << (9 * 2));
    /* Output mode configuration*/
    GPIOB->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)(9)));
    GPIOB->OTYPER |= (((uint16_t)(0x01)) << ((uint16_t)(9)));
    /* Pull-up Pull down resistor configuration*/
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)(9) * 2));
    GPIOB->PUPDR |= (((uint32_t)(0x00)) << (9 * 2));

    // 直接设置GPIO B8为AF4
    GPIOB->AFR[1] &= ~(0xF << 0); // 清除B8的AF设置
    GPIOB->AFR[1] |= (0x4 << 0);  // 设置B8为AF4
    
    // 直接设置GPIO B9为AF4
    GPIOB->AFR[1] &= ~(0xF << 4); // 清除B9的AF设置
    GPIOB->AFR[1] |= (0x4 << 4);  // 设置B9为AF4

    // 复位I2C1
    RCC->APB1RSTR |= (1 << 21);    // I2C1 复位使能
    RCC->APB1RSTR &= ~(1 << 21);   // I2C1 复位解除

    // I2C1 CR2配置 - 设置时钟频率
    I2C1->CR2 &= ~I2C_CR2_FREQ;    // 清除频率位
    I2C1->CR2 |= 8;                // 8MHz

    // I2C1 CCR配置
    I2C1->CR1 &= ~I2C_CR1_PE;      // 禁用I2C
    
    // 设置CCR寄存器 - 400KHz快速模式
    I2C1->CCR = (uint16_t)(0x0007 | I2C_CCR_FS); // 8MHz/(400KHz*3) + 快速模式
    
    // 设置上升时间
    I2C1->TRISE = 3;               // (8*300/1000)+1 = 3.4，取整为3
    
    // 使能I2C
    I2C1->CR1 |= I2C_CR1_PE;

    // I2C1 CR1配置
    I2C1->CR1 &= 0xFBF5;           // 清除ACK, SMBTYPE和SMBUS位
    I2C1->CR1 |= 0x0400;           // 使能ACK

    // I2C1 OAR1配置
    I2C1->OAR1 = 0x4030;           // 0x4000 | I2C_SLAVE_ADDRESS7(0x30)

    // 使能I2C错误中断
    I2C1->CR2 |= 0x0100;           // 使能错误中断

    NVIC->IP[I2C1_EV_IRQn] = 0x70; // 优先级7
    NVIC->ISER[I2C1_EV_IRQn >> 5] = (1 << (I2C1_EV_IRQn & 0x1F)); // 使能中断
    NVIC->IP[I2C3_ER_IRQn] = 0x70; // 优先级7
    NVIC->ISER[I2C3_ER_IRQn >> 5] = (1 << (I2C3_ER_IRQn & 0x1F)); // 使能中断

	i2c->isBusFreeSemaphore = xSemaphoreCreateBinary();
	i2c->isBusFreeMutex = xSemaphoreCreateMutex();
}

//解锁IIC总线
static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA)
{
	// 设置SDA引脚为高电平 - 使用SET位操作
	portSDA->BSRRL = pinSDA; // 在STM32F4中使用BSRRL代替BSRR的高16位操作
	
	/* Check SDA line to determine if slave is asserting bus and clock out if so */
	while((portSDA->IDR & pinSDA) == 0)
	{
		/* Set clock high */
		portSCL->BSRRL = pinSCL; // 在STM32F4中使用BSRRL代替BSRR的高16位操作
		/* Wait for any clock stretching to finish. */
		GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, I2CDEV_LOOPS_PER_MS);
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

		/* Generate a clock cycle */
		portSCL->BSRRH = pinSCL; // 在STM32F4中使用BSRRH代替BSRR的低16位操作
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
		portSCL->BSRRL = pinSCL; // 在STM32F4中使用BSRRL代替BSRR的高16位操作
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	}

	/* Generate a start then stop condition */
	portSCL->BSRRL = pinSCL; // 在STM32F4中使用BSRRL代替BSRR的高16位操作
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	portSDA->BSRRH = pinSDA; // 在STM32F4中使用BSRRH代替BSRR的低16位操作
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	portSDA->BSRRH = pinSDA; // 在STM32F4中使用BSRRH代替BSRR的低16位操作
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

	/* Set data and clock high and wait for any clock stretching to finish. */
	portSDA->BSRRL = pinSDA; // 在STM32F4中使用BSRRL代替BSRR的高16位操作
	portSCL->BSRRL = pinSCL; // 在STM32F4中使用BSRRL代替BSRR的高16位操作
	GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, I2CDEV_LOOPS_PER_MS);
	/* Wait for data to be high */
	GPIO_WAIT_FOR_HIGH(portSDA, pinSDA, I2CDEV_LOOPS_PER_MS);
}

//IIC外设初始化
void i2cdrvInit(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}

//创建一个用于传输内部寄存器的信息
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


//I2C轮询方式写数据
static bool i2cWritePoll(I2cDrv* i2c, I2cMessage* message)
{
    uint16_t timeout;
    uint16_t i;

    // 发送起始位
    I2C1->CR1 |= I2C_CR1_START;
    
    // 等待起始位发送完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0) return false;
    }
    
    // 发送设备地址(写)
    I2C1->DR = message->slaveAddress << 1;
    
    // 等待地址发送完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (--timeout == 0) return false;
        // 检查是否有ACK错误
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF; // 清除AF标志
            I2C1->CR1 |= I2C_CR1_STOP; // 发送停止位
            return false;
        }
    }
    
    // 清除ADDR标志
    uint16_t temp = I2C1->SR2;
    (void)temp;
    
    // 如果有内部地址，先发送内部地址
    if (message->internalAddress != I2C_NO_INTERNAL_ADDRESS) {
        if (message->isInternal16bit) {
            // 发送内部地址高字节
            I2C1->DR = (message->internalAddress & 0xFF00) >> 8;
            timeout = I2C_MESSAGE_TIMEOUT;
            while (!(I2C1->SR1 & I2C_SR1_TXE)) {
                if (--timeout == 0) return false;
            }
        }
        
        // 发送内部地址低字节
        I2C1->DR = message->internalAddress & 0x00FF;
        timeout = I2C_MESSAGE_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_TXE)) {
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
            I2C1->DR = message->buffer[i];
            
            timeout = I2C_MESSAGE_TIMEOUT;
            while (!(I2C1->SR1 & I2C_SR1_TXE)) {
                if (--timeout == 0) return false;
            }
        }
    }
    
    // 等待传输完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if (--timeout == 0) return false;
    }
    
    // 发送停止位
    I2C1->CR1 |= I2C_CR1_STOP;
    
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
    I2C1->CR1 |= I2C_CR1_START;
    
    // 等待起始位发送完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0) return false;
    }
    
    // 发送设备地址(读)
    I2C1->DR = (message->slaveAddress << 1) | 0x01;
    
    // 等待地址发送完成
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (--timeout == 0) return false;
        // 检查是否有ACK错误
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF; // 清除AF标志
            I2C1->CR1 |= I2C_CR1_STOP; // 发送停止位
            return false;
        }
    }
    
    // 配置ACK
    if (message->messageLength == 1) {
        // 单字节读取，需要在清除ADDR前禁用ACK
        I2C1->CR1 &= ~I2C_CR1_ACK;
    } else {
        // 多字节读取，启用ACK
        I2C1->CR1 |= I2C_CR1_ACK;
    }
    
    // 清除ADDR标志
    uint16_t temp = I2C1->SR2;
    (void)temp;
    
    // 读取数据
    for (i = 0; i < message->messageLength; i++) {
        if (i == message->messageLength - 1) {
            // 最后一个字节发送NACK
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }
        
        if (i == message->messageLength - 1) {
            // 最后一个字节之前发送停止位
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        
        // 等待接收数据
        timeout = I2C_MESSAGE_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
            if (--timeout == 0) return false;
        }
        
        // 读取数据
        message->buffer[i] = I2C1->DR;
    }
    
    return true;
}

//发送或者接收IIC信息(轮询方式)
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
