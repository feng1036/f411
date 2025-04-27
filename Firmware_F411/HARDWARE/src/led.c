#include <stdbool.h>
#include "sys.h"
#include "led.h"
#include "delay.h"

/*LED ����*/
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

/* LED��ʼ�� */
void ledInit(void)
{
    if(isInit)	return;

    /*ʹ��ledʱ��*/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOAʱ��ʹ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIOBʱ��ʹ��
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // GPIOCʱ��ʹ��

    /*LED_GREEN_L PA6	LED_RED_L PA7*/
    // ����Ϊ���ģʽ
    GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0); // ���ģʽ(01)
    // ����Ϊ�������
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7);
    // �����ٶ�
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_0); // ����(01)

    /*LED_BLUE_L PB12*/
    // ����Ϊ���ģʽ
    GPIOB->MODER &= ~GPIO_MODER_MODER12;
    GPIOB->MODER |= GPIO_MODER_MODER12_0; // ���ģʽ(01)
    // ����Ϊ�������
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_12;
    // �����ٶ�
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR12;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_0; // ����(01)

    /*LED_GREEN_R PC13	LED_RED_R PC14*/
    // ����Ϊ���ģʽ
    GPIOC->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER14);
    GPIOC->MODER |= (GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0); // ���ģʽ(01)
    // ����Ϊ�������
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14);
    // �����ٶ�
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14);
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR13_0 | GPIO_OSPEEDER_OSPEEDR14_0); // ����(01)

    ledClearAll();

    isInit = true;
}

/* LED���� */
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

/*�ر�����LED*/
void ledClearAll(void)
{
    for(u8 i = 0; i < LED_NUM; i++)
    {
        ledSet((led_e)i, 0);
    }
}

/*������LED*/
void ledSetAll(void)
{
    for(u8 i = 0; i < LED_NUM; i++)
    {
        ledSet((led_e)i, 1);
    }
}
/*LED��˸1��*/
void ledFlashOne(led_e led, u32 onTime, u32 offTime)
{
    ledSet(led, 1);
    delay_xms(onTime);
    ledSet(led, 0);
    delay_xms(offTime);
}

/* ����ĳ��LED��״̬ */
void ledSet(led_e led, bool value)
{
    if (led > LED_NUM)
        return;

    if (leds[led].polarity == LED_POL_NEG)
        value = !value;

    if(value)
        leds[led].port->BSRRL = leds[led].pin; // ��λ
    else
        leds[led].port->BSRRH = leds[led].pin; // ��λ
}
