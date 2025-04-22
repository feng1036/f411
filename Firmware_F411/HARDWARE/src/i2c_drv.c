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
	while(((gpio->IDR & pin) == 0) && i--);\
}

#define GPIO_WAIT_FOR_LOW(gpio, pin, timeoutcycles) \
{\
	int i = timeoutcycles;\
	while(((gpio->IDR & pin) != 0) && i--);\
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

// static const I2cDef deckBusDef =
// {
// 	.i2cPort            = I2C3,
// 	.i2cPerif           = RCC_APB1Periph_I2C3,
// 	.i2cEVIRQn          = I2C3_EV_IRQn,
// 	.i2cERIRQn          = I2C3_ER_IRQn,
// 	.i2cClockSpeed      = I2C_DECK_CLOCK_SPEED,
// 	.gpioSCLPerif       = RCC_AHB1Periph_GPIOA,
// 	.gpioSCLPort        = GPIOA,
// 	.gpioSCLPin         = GPIO_Pin_8,
// 	.gpioSCLPinSource   = GPIO_PinSource8,
// 	.gpioSDAPerif       = RCC_AHB1Periph_GPIOB,
// 	.gpioSDAPort        = GPIOB,
// 	.gpioSDAPin         = GPIO_Pin_4,
// 	.gpioSDAPinSource   = GPIO_PinSource4,
// 	.gpioAF             = GPIO_AF_I2C3,
// };

// I2cDrv deckBus =
// {
// 	.def                = &deckBusDef,
// };


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
	// I2C_InitTypeDef  I2C_InitStructure;
	// NVIC_InitTypeDef NVIC_InitStructure;
	// GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIOA clock
	// RCC_AHB1PeriphClockCmd(i2c->def->gpioSDAPerif, ENABLE);
    RCC->AHB1ENR |= ((uint32_t)0x00000002);
    // RCC->AHB1ENR |= i2c->def->gpioSDAPerif;
	// RCC_AHB1PeriphClockCmd(i2c->def->gpioSCLPerif, ENABLE);
    RCC->AHB1ENR |= ((uint32_t)0x00000002);
    // RCC->AHB1ENR |= i2c->def->gpioSCLPerif;
	// Enable I2C_SENSORS clock
	// RCC_APB1PeriphClockCmd(i2c->def->i2cPerif, ENABLE);	
    RCC->APB1ENR |= ((uint32_t)0x00200000);

	// GPIO_StructInit(&GPIO_InitStructure);
	// GPIO_InitStructure.GPIO_Mode = 0x01;
	// GPIO_InitStructure.GPIO_Speed = 0x02;
	// GPIO_InitStructure.GPIO_OType = 0x01;
	// GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0100); // SCL

	// GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
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

	// GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; // SDA
	// GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);
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

	// Configure I2C_SENSORS pins for AF.
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	// GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; // SCL
	// GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
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

	// GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; // SDA
	// GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);
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

	//Map gpios to alternate functions
	// GPIO_PinAFConfig(i2c->def->gpioSCLPort, i2c->def->gpioSCLPinSource, i2c->def->gpioAF);
    uint32_t temp = 0x00;
    uint32_t temp_2 = 0x00;
    temp = ((uint32_t)((uint8_t)0x04) << ((uint32_t)((uint32_t)((uint8_t)0x08) & (uint32_t)0x07) * 4)) ;
    GPIOB->AFR[((uint8_t)0x08) >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)((uint8_t)0x08) & (uint32_t)0x07) * 4)) ;
    temp_2 = GPIOB->AFR[((uint8_t)0x08) >> 0x03] | temp;
    GPIOB->AFR[((uint8_t)0x08) >> 0x03] = temp_2;
	// GPIO_PinAFConfig(i2c->def->gpioSDAPort, i2c->def->gpioSDAPinSource, i2c->def->gpioAF);
    temp = 0x00;
    temp_2 = 0x00;
    temp = ((uint32_t)((uint8_t)0x04) << ((uint32_t)((uint32_t)((uint8_t)0x09) & (uint32_t)0x07) * 4)) ;
    GPIOB->AFR[((uint8_t)0x09) >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)((uint8_t)0x09) & (uint32_t)0x07) * 4)) ;
    temp_2 = GPIOB->AFR[((uint8_t)0x09) >> 0x03] | temp;
    GPIOB->AFR[((uint8_t)0x09) >> 0x03] = temp_2;

	// I2C_SENSORS configuration
	// I2C_DeInit(i2c->def->i2cPort);
    // RCC_APB1PeriphResetCmd(((uint32_t)0x00200000), ENABLE);
    RCC->APB1RSTR |= ((uint32_t)0x00200000);
    /* Release I2C1 from reset state */
    // RCC_APB1PeriphResetCmd(((uint32_t)0x00200000), DISABLE);    
    RCC->APB1RSTR &= ~((uint32_t)0x00200000);

	// I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	// I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	// I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
	// I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	// I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// I2C_InitStructure.I2C_ClockSpeed = i2c->def->i2cClockSpeed;

    // I2C_Init(i2c->def->i2cPort, &I2C_InitStructure);
    uint16_t tmpreg = 0, freqrange = 0;
    uint16_t result = 0x04;
    uint32_t pclk1 = 8000000;

    /*---------------------------- I2C1 CR2 Configuration ------------------------*/
    /* Get the I2C1 CR2 value */
    tmpreg = I2C1->CR2;
    /* Clear frequency FREQ[5:0] bits */
    tmpreg &= (uint16_t)~((uint16_t)I2C_CR2_FREQ);
    
    /* 直接计算PCLK1时钟频率，而不是调用RCC_GetClocksFreq */
    /* 获取系统时钟源 */
    // uint32_t sysclk;
    // uint32_t tmp;
    uint32_t cfgr = RCC->CFGR;
    // uint8_t sws = (cfgr & RCC_CFGR_SWS) >> 2;
    
    // // 确定系统时钟源
    // if (sws == 0) { // HSI作为系统时钟
    //     sysclk = HSI_VALUE;
    // } else if (sws == 1) { // HSE作为系统时钟
    //     sysclk = HSE_VALUE;
    // } else { // PLL作为系统时钟
    //     // 确定PLL输入时钟源
    //     // RCC_CFGR_PLLSRC在STM32F4中是位22，修正为RCC_PLLCFGR_PLLSRC
    //     uint32_t pllsrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
    //     uint32_t pllm = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 0;
    //     uint32_t plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
    //     uint32_t pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
        
    //     if (pllsrc == 0) { // HSI作为PLL输入
    //         sysclk = (HSI_VALUE / pllm) * plln / pllp;
    //     } else { // HSE作为PLL输入
    //         sysclk = (HSE_VALUE / pllm) * plln / pllp;
    //     }
    // }
    
    // // 计算PCLK1频率
    // tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
    // if (tmp & 0x4) { // PPRE1[2] == 1, 有分频
    //     tmp = (tmp & 0x3) + 1; // 获取分频系数，+1是因为PPRE1是0b100开始的
    //     pclk1 = sysclk >> tmp; // 右移相当于除以2^tmp
    // } else {
    //     pclk1 = sysclk; // 无分频
    // }
    
    /* Set frequency bits depending on pclk1 value */
    freqrange = (uint16_t)(pclk1 / 1000000);
    tmpreg |= freqrange;
    /* Write to I2C1 CR2 */
    I2C1->CR2 = tmpreg;

    /*---------------------------- I2C1 CCR Configuration ------------------------*/
    /* Disable the selected I2C peripheral to configure TRISE */
    I2C1->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_PE);
    /* Reset tmpreg value */
    /* Clear F/S, DUTY and CCR[11:0] bits */
    tmpreg = 0;

    /* Fast mode speed calculate: Tlow/Thigh = 2 */
    result = (uint16_t)(pclk1 / (I2C_SENSORS_CLOCK_SPEED * 3));

    /* Test if CCR value is under 0x1*/
    if ((result & I2C_CCR_CCR) == 0)
    {
    /* Set minimum allowed value */
    result |= (uint16_t)0x0001;  
    }
    /* Set speed value and set F/S bit for fast mode */
    tmpreg |= (uint16_t)(result | I2C_CCR_FS);
    /* Set Maximum Rise Time for fast mode */
    I2C1->TRISE = (uint16_t)(((freqrange * (uint16_t)300) / (uint16_t)1000) + (uint16_t)1);  

    /* Write to I2C1 CCR */
    I2C1->CCR = tmpreg;
    /* Enable the selected I2C peripheral */
    I2C1->CR1 |= I2C_CR1_PE;

    /*---------------------------- I2C1 CR1 Configuration ------------------------*/
    /* Get the I2C1 CR1 value */
    tmpreg = I2C1->CR1;
    /* Clear ACK, SMBTYPE and  SMBUS bits */
    tmpreg &= ((uint16_t)0xFBF5);
    /* Configure I2C1: mode and acknowledgement */
    /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
    /* Set ACK bit according to I2C_Ack value */
    tmpreg |= (uint16_t)((uint32_t)(0x0000) | ((uint16_t)0x0400));
    /* Write to I2C1 CR1 */
    I2C1->CR1 = tmpreg;

    /*---------------------------- I2C1 OAR1 Configuration -----------------------*/
    /* Set I2C1 Own Address1 and acknowledged address */
    I2C1->OAR1 = (((uint16_t)0x4000) | I2C_SLAVE_ADDRESS7);
    

	// Enable I2C_SENSORS error interrupts
	// I2C_ITConfig(i2c->def->i2cPort, I2C_IT_ERR, ENABLE);
    I2C1->CR2 |= ((uint16_t)0x0100);

	// NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);
    uint8_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;
    /* Compute the Corresponding IRQ Priority --------------------------------*/    
    tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;
    tmppriority = 7 << tmppre;
    tmppriority |=  (uint8_t)(7 & tmpsub);
    tmppriority = tmppriority << 0x04;
    NVIC->IP[I2C1_EV_IRQn] = tmppriority;
    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[I2C1_EV_IRQn >> 0x05] =
      (uint32_t)0x01 << (I2C1_EV_IRQn & (uint8_t)0x1F);

	
	// NVIC_InitStructure.NVIC_IRQChannel = I2C3_ER_IRQn;
	// NVIC_Init(&NVIC_InitStructure);
    tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;
    /* Compute the Corresponding IRQ Priority --------------------------------*/    
    tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;
    tmppriority = 7 << tmppre;
    tmppriority |=  (uint8_t)(7 & tmpsub);
    tmppriority = tmppriority << 0x04;
    NVIC->IP[I2C3_ER_IRQn] = tmppriority;
    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[I2C3_ER_IRQn >> 0x05] =
      (uint32_t)0x01 << (I2C3_ER_IRQn & (uint8_t)0x1F);
	

	i2c->isBusFreeSemaphore = xSemaphoreCreateBinary();
	i2c->isBusFreeMutex = xSemaphoreCreateMutex();
}
/**
 * 解锁IIC总线
 */
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
