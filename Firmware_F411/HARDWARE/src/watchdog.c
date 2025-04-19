#include "config.h"
#include "watchdog.h"
#include "debug_assert.h"


bool watchdogTest(void)
{
	bool wasNormalStart = true;

	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST)) 
	{
		RCC_ClearFlag();
		wasNormalStart = false;
		printAssertSnapshotData();
	}
	return wasNormalStart;
}


void watchdogInit(u16 xms)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	IWDG_SetReload((u16)(1.47*xms));

	watchdogReset();
	IWDG_Enable();
}
