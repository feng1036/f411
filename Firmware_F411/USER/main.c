#include "system.h"	/*头文件集合*/

TaskHandle_t startTaskHandle;
static void startTask(void *arg);

int main()
{
							
    systemInit();			/*底层硬件初始化*/

    xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*创建起始任务*/

    vTaskStartScheduler();	/*开启任务调度*/	

    while(1) {};
}
/*创建任务*/
void startTask(void *arg)
{
    taskENTER_CRITICAL();	/*进入临界区*/

    //xTaskCreate(radiolinkTask, "RADIOLINK", 150, NULL, 5, NULL);		/*创建无线连接任务*/

    xTaskCreate(sensorsTask, "SENSORS", 450, NULL, 4, NULL);			/*创建传感器处理任务*/

    xTaskCreate(stabilizerTask, "STABILIZER", 450, NULL, 5, NULL);		/*创建姿态任务*/

    vTaskDelete(startTaskHandle);										/*删除开始任务*/

    taskEXIT_CRITICAL();	/*退出临界区*/
}

void vApplicationIdleHook( void )
{
    static u32 tickWatchdogReset = 0;

    portTickType tickCount = getSysTickCnt();

    if (tickCount - tickWatchdogReset > WATCHDOG_RESET_MS)
    {
        tickWatchdogReset = tickCount;
        watchdogReset();
    }

    __WFI();	/*进入低功耗模式*/
}
