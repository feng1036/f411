#include <string.h>
#include "stm32f4xx.h" 
#include "i2c_drv.h"
#include "delay.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

 // IIC�ײ�������ʼ��
static void i2cdrvInitBus(I2cDrv* i2c);

//����IIC����
static void i2cdrvTryToRestartBus(I2cDrv* i2c);

//��·��ʱ
static inline void i2cdrvRoughLoopDelay(uint32_t us);

//����IIC����
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

//����������
I2cDrv sensorsBus =
{
	.def                = &sensorsBusDef,
};

//��·��ʱ
static inline void i2cdrvRoughLoopDelay(uint32_t us)
{
	volatile uint32_t delay = 0;
	for(delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay) { };
}

//����IIC����
static void i2cdrvTryToRestartBus(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}

//IIC�ײ�������ʼ��
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

    // ֱ������GPIO B8ΪAF4
    GPIOB->AFR[1] &= ~(0xF << 0); // ���B8��AF����
    GPIOB->AFR[1] |= (0x4 << 0);  // ����B8ΪAF4
    
    // ֱ������GPIO B9ΪAF4
    GPIOB->AFR[1] &= ~(0xF << 4); // ���B9��AF����
    GPIOB->AFR[1] |= (0x4 << 4);  // ����B9ΪAF4

    // ��λI2C1
    RCC->APB1RSTR |= (1 << 21);    // I2C1 ��λʹ��
    RCC->APB1RSTR &= ~(1 << 21);   // I2C1 ��λ���

    // I2C1 CR2���� - ����ʱ��Ƶ��
    I2C1->CR2 &= ~I2C_CR2_FREQ;    // ���Ƶ��λ
    I2C1->CR2 |= 8;                // 8MHz

    // I2C1 CCR����
    I2C1->CR1 &= ~I2C_CR1_PE;      // ����I2C
    
    // ����CCR�Ĵ��� - 400KHz����ģʽ
    I2C1->CCR = (uint16_t)(0x0007 | I2C_CCR_FS); // 8MHz/(400KHz*3) + ����ģʽ
    
    // ��������ʱ��
    I2C1->TRISE = 3;               // (8*300/1000)+1 = 3.4��ȡ��Ϊ3
    
    // ʹ��I2C
    I2C1->CR1 |= I2C_CR1_PE;

    // I2C1 CR1����
    I2C1->CR1 &= 0xFBF5;           // ���ACK, SMBTYPE��SMBUSλ
    I2C1->CR1 |= 0x0400;           // ʹ��ACK

    // I2C1 OAR1����
    I2C1->OAR1 = 0x4030;           // 0x4000 | I2C_SLAVE_ADDRESS7(0x30)

    // ʹ��I2C�����ж�
    I2C1->CR2 |= 0x0100;           // ʹ�ܴ����ж�

    NVIC->IP[I2C1_EV_IRQn] = 0x70; // ���ȼ�7
    NVIC->ISER[I2C1_EV_IRQn >> 5] = (1 << (I2C1_EV_IRQn & 0x1F)); // ʹ���ж�
    NVIC->IP[I2C3_ER_IRQn] = 0x70; // ���ȼ�7
    NVIC->ISER[I2C3_ER_IRQn >> 5] = (1 << (I2C3_ER_IRQn & 0x1F)); // ʹ���ж�

	i2c->isBusFreeSemaphore = xSemaphoreCreateBinary();
	i2c->isBusFreeMutex = xSemaphoreCreateMutex();
}

//����IIC����
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

//IIC�����ʼ��
void i2cdrvInit(I2cDrv* i2c)
{
	i2cdrvInitBus(i2c);
}

//����һ�����ڴ����ڲ��Ĵ�������Ϣ
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


//I2C��ѯ��ʽд����
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

//���ͻ��߽���IIC��Ϣ(��ѯ��ʽ)
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
