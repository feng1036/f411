#include <stdbool.h>
#include "sys.h"
#include "led.h"
#include "delay.h"

/*LED 极性*/
#define LED_POL_POS 0
#define LED_POL_NEG 1

static bool isInit = false;

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
    int polarity;
} led_t;

static led_t leds[LED_NUM] =
{
    [LED_BLUE_L]	= {GPIOB, (1<<12), LED_POL_POS},
    [LED_GREEN_L]	= {GPIOA, (1<<6),  LED_POL_NEG},
    [LED_RED_L] 	= {GPIOA, (1<<7),  LED_POL_NEG},
    [LED_GREEN_R]	= {GPIOC, (1<<13), LED_POL_NEG},
    [LED_RED_R] 	= {GPIOC, (1<<14), LED_POL_NEG},
};

/* LED初始化 */
void ledInit(void)
{
    if(isInit)	return;

    /*使能led时钟*/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA时钟使能
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIOB时钟使能
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // GPIOC时钟使能

    /*LED_GREEN_L PA6	LED_RED_L PA7*/
    // 设置为输出模式
    GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0); // 输出模式(01)
    // 设置为推挽输出
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7);
    // 设置速度
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_0); // 中速(01)

    /*LED_BLUE_L PB12*/
    // 设置为输出模式
    GPIOB->MODER &= ~GPIO_MODER_MODER12;
    GPIOB->MODER |= GPIO_MODER_MODER12_0; // 输出模式(01)
    // 设置为推挽输出
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_12;
    // 设置速度
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR12;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_0; // 中速(01)

    /*LED_GREEN_R PC13	LED_RED_R PC14*/
    // 设置为输出模式
    GPIOC->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER14);
    GPIOC->MODER |= (GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0); // 输出模式(01)
    // 设置为推挽输出
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14);
    // 设置速度
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14);
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR13_0 | GPIO_OSPEEDER_OSPEEDR14_0); // 中速(01)

    ledClearAll();

    isInit = true;
}

/* LED测试 */
bool ledTest(void)
{
    ledSet(LED_GREEN_L, 1);
    ledSet(LED_GREEN_R, 1);
    ledSet(LED_RED_L, 0);
    ledSet(LED_RED_R, 0);
    delay_xms(250);

    ledSet(LED_GREEN_L, 0);
    ledSet(LED_GREEN_R, 0);
    ledSet(LED_RED_L, 1);
    ledSet(LED_RED_R, 1);

    delay_xms(250);

    ledClearAll();
    ledSet(LED_BLUE_L, 1);

    return isInit;
}

/*关闭所有LED*/
void ledClearAll(void)
{
    for(u8 i = 0; i < LED_NUM; i++)
    {
        ledSet((led_e)i, 0);
    }
}

/*打开所有LED*/
void ledSetAll(void)
{
    for(u8 i = 0; i < LED_NUM; i++)
    {
        ledSet((led_e)i, 1);
    }
}
/*LED闪烁1次*/
void ledFlashOne(led_e led, u32 onTime, u32 offTime)
{
    ledSet(led, 1);
    delay_xms(onTime);
    ledSet(led, 0);
    delay_xms(offTime);
}

/* 设置某个LED的状态 */
void ledSet(led_e led, bool value)
{
    if (led > LED_NUM)
        return;

    if (leds[led].polarity == LED_POL_NEG)
        value = !value;

    if(value)
        leds[led].port->BSRRL = leds[led].pin; // 置位
    else
        leds[led].port->BSRRH = leds[led].pin; // 复位
}
