#include "system.h"	/*ͷ�ļ�����*/

TaskHandle_t startTaskHandle;
static void startTask(void *arg);

int main()
{
							
    systemInit();			/*�ײ�Ӳ����ʼ��*/

    xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*������ʼ����*/

    vTaskStartScheduler();	/*�����������*/	

    while(1) {};
}
/*��������*/
void startTask(void *arg)
{
    taskENTER_CRITICAL();	/*�����ٽ���*/

    //xTaskCreate(radiolinkTask, "RADIOLINK", 150, NULL, 5, NULL);		/*����������������*/

    xTaskCreate(sensorsTask, "SENSORS", 450, NULL, 4, NULL);			/*������������������*/

    xTaskCreate(stabilizerTask, "STABILIZER", 450, NULL, 5, NULL);		/*������̬����*/

    vTaskDelete(startTaskHandle);										/*ɾ����ʼ����*/

    taskEXIT_CRITICAL();	/*�˳��ٽ���*/
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

    __WFI();	/*����͹���ģʽ*/
}
