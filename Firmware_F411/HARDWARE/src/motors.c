#include "sys.h"
#include "delay.h"
#include "motors.h"

static bool isInit = false;
u32 motor_ratios[] = {0, 0, 0, 0};
static const u32 MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

static u16 ratioToCCRx(u16 val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

void motorsInit(void)	/*电机初始化*/
{
	// 使能GPIO和定时器时钟
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;
	
	// 复位定时器
	TIM4->CR1 = 0;
	TIM4->CCER = 0;
	TIM4->CCMR1 = 0;
	TIM4->CCMR2 = 0;
	TIM4->CNT = 0;
	TIM4->ARR = 0;
	TIM4->PSC = 0;
	
	TIM2->CR1 = 0;
	TIM2->CCER = 0;
	TIM2->CCMR1 = 0;
	TIM2->CCMR2 = 0;
	TIM2->CNT = 0;
	TIM2->ARR = 0;
	TIM2->PSC = 0;
	
	// 配置引脚复用功能
	// PB7->TIM4 CH2, PB6->TIM4 CH1, PB10->TIM2 CH3, PA5->TIM2 CH1
	
	// 配置PB7为TIM4 CH2 (MOTOR1)
	GPIOB->MODER &= ~GPIO_MODER_MODER7;
	GPIOB->MODER |= GPIO_MODER_MODER7_1;  // 设置为复用功能
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_7;   // 推挽输出
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7; // 100MHz
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR7;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;  // 上拉
	GPIOB->AFR[0] &= ~(0xFUL << 28); // 清除位28-31
	GPIOB->AFR[0] |= (2UL << 28);    // AF2
	
	// 配置PB6为TIM4 CH1 (MOTOR2)
	GPIOB->MODER &= ~GPIO_MODER_MODER6;
	GPIOB->MODER |= GPIO_MODER_MODER6_1;  // 设置为复用功能
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_6;   // 推挽输出
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6; // 100MHz
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR6;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;  // 上拉
	GPIOB->AFR[0] &= ~(0xFUL << 24); // 清除位24-27
	GPIOB->AFR[0] |= (2UL << 24);    // AF2
	
	// 配置PB10为TIM2 CH3 (MOTOR3)
	GPIOB->MODER &= ~GPIO_MODER_MODER10;
	GPIOB->MODER |= GPIO_MODER_MODER10_1; // 设置为复用功能
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_10;  // 推挽输出
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10; // 100MHz
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR10;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0; // 上拉
	GPIOB->AFR[1] &= ~(0xFUL << 8);  // 清除位8-11 (对应PB10)
	GPIOB->AFR[1] |= (1UL << 8);     // AF1
	
	// 配置PA5为TIM2 CH1 (MOTOR4)
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER5_1;  // 设置为复用功能
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;   // 推挽输出
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5; // 100MHz
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_0;  // 上拉
	GPIOA->AFR[0] &= ~(0xFUL << 20); // 清除位20-23
	GPIOA->AFR[0] |= (1UL << 20);    // AF1
	
	// 配置定时器基础参数
	TIM4->PSC = MOTORS_PWM_PRESCALE;     // 预分频器
	TIM4->ARR = MOTORS_PWM_PERIOD;       // 自动重装载值
	TIM2->PSC = MOTORS_PWM_PRESCALE;     // 预分频器
	TIM2->ARR = MOTORS_PWM_PERIOD;       // 自动重装载值
	
	// 配置TIM4 CH1(PB6, MOTOR2)
	TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM模式1
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;      // 预装载使能
	TIM4->CCER |= TIM_CCER_CC1E;         // 输出使能
	TIM4->CCR1 = 0;                      // 初始占空比为0
	
	// 配置TIM4 CH2(PB7, MOTOR1)
	TIM4->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // PWM模式1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;      // 预装载使能
	TIM4->CCER |= TIM_CCER_CC2E;         // 输出使能
	TIM4->CCR2 = 0;                      // 初始占空比为0
	
	// 配置TIM2 CH3(PB10, MOTOR3)
	TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2); // PWM模式1
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;      // 预装载使能
	TIM2->CCER |= TIM_CCER_CC3E;         // 输出使能
	TIM2->CCR3 = 0;                      // 初始占空比为0
	
	// 配置TIM2 CH1(PA5, MOTOR4)
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM模式1
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;      // 预装载使能
	TIM2->CCER |= TIM_CCER_CC1E;         // 输出使能
	TIM2->CCR1 = 0;                      // 初始占空比为0
	
	// 启用ARR预装载
	TIM4->CR1 |= TIM_CR1_ARPE;
	TIM2->CR1 |= TIM_CR1_ARPE;
	
	// 启动定时器
	TIM4->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 |= TIM_CR1_CEN;

	isInit = true;
}

/*电机测试*/
bool motorsTest(void)
{
	int i;
	
	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{	
		motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
		delay_xms(MOTORS_TEST_ON_TIME_MS);
		motorsSetRatio(MOTORS[i], 0);
		delay_xms(MOTORS_TEST_DELAY_TIME_MS);
	}

	return isInit;
}

/*设置电机PWM占空比*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
		u16 ratio=ithrust;

	#ifdef ENABLE_THRUST_BAT_COMPENSATED		
		float thrust = ((float)ithrust / 65536.0f) * 60;
		float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
		float supply_voltage = 3.7f;
		float percentage = volts / supply_voltage;
		percentage = percentage > 1.0f ? 1.0f : percentage;
		ratio = percentage * UINT16_MAX;
		motor_ratios[id] = ratio;
	#endif
		
		u16 ccr_value = ratioToCCRx(ratio);
		
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM4->CCR2 = ccr_value;
				break;
			case 1:		/*MOTOR_M2*/
				TIM4->CCR1 = ccr_value;
				break;
			case 2:		/*MOTOR_M3*/
				TIM2->CCR3 = ccr_value;
				break;
			case 3:		/*MOTOR_M4*/	
				TIM2->CCR1 = ccr_value;
				break;
			default: break;
		}	
	}
}

