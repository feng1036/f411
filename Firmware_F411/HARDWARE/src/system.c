#include "system.h"


static bool systemTest(void);

/*底层硬件初始化*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*中断配置初始化*/
	extiInit();			/*外部中断初始化*/	
	delay_init(96);		/*delay初始化*/
	ledInit();			/*led初始化*/
	ledseqInit();		/*led灯序列初始化*/
	communicateInit();	/*和stablilizer通信的初始化*/
	uartslkInit();		/*串口初始化*/
	pmInit();			/*电源管理初始化*/
	stabilizerInit();	/*电机 传感器 PID初始化*/
	if(systemTest() == true)
	{	
		while(cnt++ < 5)	/*初始化通过 左上绿灯快闪5次*/
		{
			ledFlashOne(LED_GREEN_L, 50, 50);
		}			
	}
	watchdogInit(WATCHDOG_RESET_MS);	/*看门狗初始化*/
}

static bool systemTest(void)
{
	bool pass = true;
	return pass;
}

