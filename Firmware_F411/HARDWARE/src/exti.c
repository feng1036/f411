#include <stdbool.h>
#include "sys.h"
#include "exti.h"

static bool isInit;

#define NVIC_PRIORITY_GROUP_POS   (8U)
#define NVIC_PRIORITY_GROUP_MASK  (0x700U)
#define NVIC_PRIORITY_GROUP       ((SCB->AIRCR & NVIC_PRIORITY_GROUP_MASK) >> NVIC_PRIORITY_GROUP_POS)
void NVIC_ConfigureIRQ(IRQn_Type IRQn, uint8_t preempt_prio)
{
	uint32_t prigroup = NVIC_PRIORITY_GROUP;
    //uint8_t pre_bits = 4, sub_bits = 0;
	uint8_t priority = (preempt_prio & 0x0F);
    NVIC->IP[IRQn] = priority << 4;
    NVIC->ISER[IRQn >> 0x05] = ((uint32_t)1U << (IRQn & (uint8_t)0x1F));
}

/* Interruption initialisation */
void extiInit()
{
	//static NVIC_InitTypeDef NVIC_InitStructure;

	if (isInit)	return;
	
	//RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
	RCC->AHB2ENR |=  (uint32_t)0x00004000;
	
	NVIC_ConfigureIRQ(EXTI0_IRQn,5);

	NVIC_ConfigureIRQ(EXTI1_IRQn,12);
	
	NVIC_ConfigureIRQ(EXTI2_IRQn,12);
	
	NVIC_ConfigureIRQ(EXTI3_IRQn,12);
	
	NVIC_ConfigureIRQ(EXTI4_IRQn,12);
	
	NVIC_ConfigureIRQ(EXTI9_5_IRQn,12);
	
	NVIC_ConfigureIRQ(EXTI15_10_IRQn,10);
	isInit = true;
}

bool extiTest(void)
{
	return isInit;
}

void __attribute__((used)) EXTI0_IRQHandler(void)
{
	EXTI->PR=(uint32_t)0x00001;
	EXTI0_Callback();
}

void __attribute__((used)) EXTI1_IRQHandler(void)
{
	EXTI->PR=(uint32_t)0x00002;
	EXTI1_Callback();
}

void __attribute__((used)) EXTI2_IRQHandler(void)
{
	EXTI->PR=(uint32_t)0x00004;
	EXTI2_Callback();
}

void __attribute__((used)) EXTI3_IRQHandler(void)
{
	EXTI->PR=(uint32_t)0x00008;
	EXTI3_Callback();
}

void __attribute__((used)) EXTI4_IRQHandler(void)
{
	EXTI->PR=(uint32_t)0x00010;
	EXTI4_Callback();
}

void __attribute__((used)) EXTI9_5_IRQHandler(void)
{
	if ((EXTI->PR & (uint32_t)0x00020) != RESET) 
	{
		EXTI->PR=(uint32_t)0x00020;
		EXTI5_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x00040) != RESET) 
	{
		EXTI->PR=(uint32_t)0x00040;
		EXTI6_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x00080) != RESET)
	{
		EXTI->PR=(uint32_t)0x00080;
		EXTI7_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x00100) != RESET) 
	{
		EXTI->PR=(uint32_t)0x00100;
		EXTI8_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x00200) != RESET) 
	{
		EXTI->PR=(uint32_t)0x00200;
		EXTI9_Callback();
	}
}

void __attribute__((used)) EXTI15_10_IRQHandler(void)
{
	if ((EXTI->PR & (uint32_t)0x00400) != RESET) 
	{
		EXTI->PR=(uint32_t)0x00400;
		EXTI10_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x00800) != RESET) 
	{
		EXTI->PR=(uint32_t)0x00800;
		EXTI11_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x01000) != RESET) 
	{
		EXTI->PR=(uint32_t)0x01000;
		EXTI12_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x02000) != RESET) 
	{
		EXTI->PR=(uint32_t)0x02000;
		EXTI13_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x04000) != RESET) 
	{
		EXTI->PR=(uint32_t)0x04000;
		EXTI14_Callback();
	}
	if ((EXTI->PR & (uint32_t)0x08000) != RESET) 
	{
		EXTI->PR=(uint32_t)0x08000;
		EXTI15_Callback();
	}
}

void __attribute__((weak)) EXTI0_Callback(void) { }
void __attribute__((weak)) EXTI1_Callback(void) { }
void __attribute__((weak)) EXTI2_Callback(void) { }
void __attribute__((weak)) EXTI3_Callback(void) { }
void __attribute__((weak)) EXTI4_Callback(void) { }
void __attribute__((weak)) EXTI5_Callback(void) { }
void __attribute__((weak)) EXTI6_Callback(void) { }
void __attribute__((weak)) EXTI7_Callback(void) { }
void __attribute__((weak)) EXTI8_Callback(void) { }
void __attribute__((weak)) EXTI9_Callback(void) { }
void __attribute__((weak)) EXTI10_Callback(void) { }
void __attribute__((weak)) EXTI11_Callback(void) { }
void __attribute__((weak)) EXTI12_Callback(void) { }
void __attribute__((weak)) EXTI13_Callback(void) { }
void __attribute__((weak)) EXTI14_Callback(void) { }
void __attribute__((weak)) EXTI15_Callback(void) { }
