#include <string.h>
#include "stm32f4xx.h" 
#include "i2c_drv.h"
#include "config.h"
#include "delay.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//������IIC�����ٶ�
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
 * IIC�ײ�������ʼ��
 */
static void i2cdrvInitBus(I2cDrv* i2c);
/**
 * ����IIC����
 */
static void i2cdrvTryToRestartBus(I2cDrv* i2c);
/**
 * ��·��ʱ
 */
static inline void i2cdrvRoughLoopDelay(uint32_t us);
/**
 * ����IIC����
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
 * ����������
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
 * ��·��ʱ
 */
static inline void i2cdrvRoughLoopDelay(uint32_t us)
{
	volatile uint32_t delay = 0;
//	delay_us(us);
	for(delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay) { };
}
/**
 * ����IIC����
 */
static void i2cdrvTryToRestartBus(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}

/**
 * IIC�ײ�������ʼ��
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
    
    /* ֱ�Ӽ���PCLK1ʱ��Ƶ�ʣ������ǵ���RCC_GetClocksFreq */
    /* ��ȡϵͳʱ��Դ */
    // uint32_t sysclk;
    // uint32_t tmp;
    uint32_t cfgr = RCC->CFGR;
    // uint8_t sws = (cfgr & RCC_CFGR_SWS) >> 2;
    
    // // ȷ��ϵͳʱ��Դ
    // if (sws == 0) { // HSI��Ϊϵͳʱ��
    //     sysclk = HSI_VALUE;
    // } else if (sws == 1) { // HSE��Ϊϵͳʱ��
    //     sysclk = HSE_VALUE;
    // } else { // PLL��Ϊϵͳʱ��
    //     // ȷ��PLL����ʱ��Դ
    //     // RCC_CFGR_PLLSRC��STM32F4����λ22������ΪRCC_PLLCFGR_PLLSRC
    //     uint32_t pllsrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
    //     uint32_t pllm = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 0;
    //     uint32_t plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
    //     uint32_t pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
        
    //     if (pllsrc == 0) { // HSI��ΪPLL����
    //         sysclk = (HSI_VALUE / pllm) * plln / pllp;
    //     } else { // HSE��ΪPLL����
    //         sysclk = (HSE_VALUE / pllm) * plln / pllp;
    //     }
    // }
    
    // // ����PCLK1Ƶ��
    // tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
    // if (tmp & 0x4) { // PPRE1[2] == 1, �з�Ƶ
    //     tmp = (tmp & 0x3) + 1; // ��ȡ��Ƶϵ����+1����ΪPPRE1��0b100��ʼ��
    //     pclk1 = sysclk >> tmp; // �����൱�ڳ���2^tmp
    // } else {
    //     pclk1 = sysclk; // �޷�Ƶ
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
 * ����IIC����
 */
static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA)
{
	// ����SDA����Ϊ�ߵ�ƽ - ʹ��SETλ����
	portSDA->BSRRL = pinSDA; // ��STM32F4��ʹ��BSRRL����BSRR�ĸ�16λ����
	
	/* Check SDA line to determine if slave is asserting bus and clock out if so */
	while((portSDA->IDR & pinSDA) == 0)
	{
		/* Set clock high */
		portSCL->BSRRL = pinSCL; // ��STM32F4��ʹ��BSRRL����BSRR�ĸ�16λ����
		/* Wait for any clock stretching to finish. */
		GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, I2CDEV_LOOPS_PER_MS);
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

		/* Generate a clock cycle */
		portSCL->BSRRH = pinSCL; // ��STM32F4��ʹ��BSRRH����BSRR�ĵ�16λ����
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
		portSCL->BSRRL = pinSCL; // ��STM32F4��ʹ��BSRRL����BSRR�ĸ�16λ����
		i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	}

	/* Generate a start then stop condition */
	portSCL->BSRRL = pinSCL; // ��STM32F4��ʹ��BSRRL����BSRR�ĸ�16λ����
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	portSDA->BSRRH = pinSDA; // ��STM32F4��ʹ��BSRRH����BSRR�ĵ�16λ����
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);
	portSDA->BSRRH = pinSDA; // ��STM32F4��ʹ��BSRRH����BSRR�ĵ�16λ����
	i2cdrvRoughLoopDelay(I2CDEV_CLK_TS);

	/* Set data and clock high and wait for any clock stretching to finish. */
	portSDA->BSRRL = pinSDA; // ��STM32F4��ʹ��BSRRL����BSRR�ĸ�16λ����
	portSCL->BSRRL = pinSCL; // ��STM32F4��ʹ��BSRRL����BSRR�ĸ�16λ����
	GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, I2CDEV_LOOPS_PER_MS);
	/* Wait for data to be high */
	GPIO_WAIT_FOR_HIGH(portSDA, pinSDA, I2CDEV_LOOPS_PER_MS);
}


/**
 * IIC�����ʼ��
 */
void i2cdrvInit(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}


/**
 * ����һ�����ڴ����ڲ��Ĵ�������Ϣ
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
 * I2C��ѯ��ʽд����
 */
static bool i2cWritePoll(I2cDrv* i2c, I2cMessage* message)
{
    uint16_t timeout;
    uint16_t i;

    // ������ʼλ
    I2C1->CR1 |= I2C_CR1_START;
    
    // �ȴ���ʼλ�������
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0) return false;
    }
    
    // �����豸��ַ(д)
    I2C1->DR = message->slaveAddress << 1;
    
    // �ȴ���ַ�������
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (--timeout == 0) return false;
        // ����Ƿ���ACK����
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF; // ���AF��־
            I2C1->CR1 |= I2C_CR1_STOP; // ����ֹͣλ
            return false;
        }
    }
    
    // ���ADDR��־
    uint16_t temp = I2C1->SR2;
    (void)temp;
    
    // ������ڲ���ַ���ȷ����ڲ���ַ
    if (message->internalAddress != I2C_NO_INTERNAL_ADDRESS) {
        if (message->isInternal16bit) {
            // �����ڲ���ַ���ֽ�
            I2C1->DR = (message->internalAddress & 0xFF00) >> 8;
            timeout = I2C_MESSAGE_TIMEOUT;
            while (!(I2C1->SR1 & I2C_SR1_TXE)) {
                if (--timeout == 0) return false;
            }
        }
        
        // �����ڲ���ַ���ֽ�
        I2C1->DR = message->internalAddress & 0x00FF;
        timeout = I2C_MESSAGE_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_TXE)) {
            if (--timeout == 0) return false;
        }
        
        // ����Ƕ���������Ҫ�����ظ���ʼλ
        if (message->direction == i2cRead) {
            return true; // ���سɹ������������ظ���ʼλ
        }
    }
    
    // ��������(ֻ��д����ʱ)
    if (message->direction == i2cWrite) {
        for (i = 0; i < message->messageLength; i++) {
            I2C1->DR = message->buffer[i];
            
            timeout = I2C_MESSAGE_TIMEOUT;
            while (!(I2C1->SR1 & I2C_SR1_TXE)) {
                if (--timeout == 0) return false;
            }
        }
    }
    
    // �ȴ��������
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if (--timeout == 0) return false;
    }
    
    // ����ֹͣλ
    I2C1->CR1 |= I2C_CR1_STOP;
    
    return true;
}

/**
 * I2C��ѯ��ʽ������
 */
static bool i2cReadPoll(I2cDrv* i2c, I2cMessage* message)
{
    uint16_t timeout;
    uint16_t i;
    
    // ������ʼλ
    I2C1->CR1 |= I2C_CR1_START;
    
    // �ȴ���ʼλ�������
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0) return false;
    }
    
    // �����豸��ַ(��)
    I2C1->DR = (message->slaveAddress << 1) | 0x01;
    
    // �ȴ���ַ�������
    timeout = I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (--timeout == 0) return false;
        // ����Ƿ���ACK����
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF; // ���AF��־
            I2C1->CR1 |= I2C_CR1_STOP; // ����ֹͣλ
            return false;
        }
    }
    
    // ����ACK
    if (message->messageLength == 1) {
        // ���ֽڶ�ȡ����Ҫ�����ADDRǰ����ACK
        I2C1->CR1 &= ~I2C_CR1_ACK;
    } else {
        // ���ֽڶ�ȡ������ACK
        I2C1->CR1 |= I2C_CR1_ACK;
    }
    
    // ���ADDR��־
    uint16_t temp = I2C1->SR2;
    (void)temp;
    
    // ��ȡ����
    for (i = 0; i < message->messageLength; i++) {
        if (i == message->messageLength - 1) {
            // ���һ���ֽڷ���NACK
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }
        
        if (i == message->messageLength - 1) {
            // ���һ���ֽ�֮ǰ����ֹͣλ
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        
        // �ȴ���������
        timeout = I2C_MESSAGE_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
            if (--timeout == 0) return false;
        }
        
        // ��ȡ����
        message->buffer[i] = I2C1->DR;
    }
    
    return true;
}

/**
 * ���ͻ��߽���IIC��Ϣ(��ѯ��ʽ)
 */
bool i2cdrvMessageTransfer(I2cDrv* i2c, I2cMessage* message)
{
    bool status = false;
    int retries = message->nbrOfRetries;

    xSemaphoreTake(i2c->isBusFreeMutex, portMAX_DELAY); // ������Ϣ����

    while (retries >= 0) {
        // д��������Ҫд�ڲ���ַ
        if (message->direction == i2cWrite || 
            message->internalAddress != I2C_NO_INTERNAL_ADDRESS) {
            status = i2cWritePoll(i2c, message);
            
            // ����ɹ����Ƕ���������Ҫ�����ظ���ʼλ���ж�ȡ
            if (status && message->direction == i2cRead) {
                status = i2cReadPoll(i2c, message);
            }
        } else {
            // ֱ�Ӷ�����
            status = i2cReadPoll(i2c, message);
        }
        
        if (status) {
            // ����ɹ�
            break;
        }
        
        retries--;
        i2cdrvRoughLoopDelay(100); // ��ʱһ��ʱ��������
    }
    
    if (!status) {
        // ��������ʧ�ܣ�������������
        i2cdrvTryToRestartBus(i2c);
    }
    
    xSemaphoreGive(i2c->isBusFreeMutex);
    return status;
}
