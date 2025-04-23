#include "exti.h"
#include "led.h"
#include "motors.h"
#include "debug_assert.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"		 
#include "task.h"

static u32 sysTickCnt=0;

void nvicInit(void)
{
	// NVIC_SetVectorTable(FIRMWARE_START_ADDR,0);
	SCB->VTOR = FIRMWARE_START_ADDR;
  	SCB->AIRCR = ((uint32_t)0x05FA0000) | ((uint32_t)0x300);
//   NVIC_PriorityGroupConfig(((uint32_t)0x300));
}

extern void xPortSysTickHandler(void);

/********************************************************
 *SysTick_Handler()
 *滴答定时器中断服务函数
*********************************************************/
void  SysTick_Handler(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
    {
        xPortSysTickHandler();	
    }else
	{
		sysTickCnt++;	/*调度开启之前计数*/
	}
}
/********************************************************
*getSysTickCnt()
*调度开启之前 返回 sysTickCnt
*调度开启之前 返回 xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}

//This function handles Hard Fault exception 
__asm void HardFault_Handler(void)
{
	PRESERVE8
    IMPORT printHardFault
    TST r14, #4
    ITE EQ
    MRSEQ R0, MSP
    MRSNE R0, PSP
    B printHardFault
}

void  printHardFault(u32* hardfaultArgs)
{
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);	/*错误检测*/
	ledSet(ERR_LED2, 1);

	storeAssertSnapshotData(__FILE__, __LINE__);
	while (1)
	{}
}
/**
 * @brief  This function handles Memory Manage exception.
 */
void  MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);/*错误检测*/
	ledSet(ERR_LED2, 1);

	storeAssertSnapshotData(__FILE__, __LINE__);
	while (1)
	{}
}

/**
 * @brief  This function handles Bus Fault exception.
 */
void  BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);/*错误检测*/
	ledSet(ERR_LED2, 1);

	storeAssertSnapshotData(__FILE__, __LINE__);
	while (1)
	{}
}

/**
 * @brief  This function handles Usage Fault exception.
 */
void  UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);/*错误检测*/
	ledSet(ERR_LED2, 1);

	storeAssertSnapshotData(__FILE__, __LINE__);
	while (1)
	{}
}
