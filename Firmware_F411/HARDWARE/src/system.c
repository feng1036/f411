#include "system.h"


/*底层硬件初始化*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*中断配置初始化*/
	extiInit();			/*外部中断初始化*/	
	delay_init(96);		/*delay初始化*/
	ledInit();			/*led初始化*/
	ledseqInit();		/*led灯序列初始化*/
	
	commInit();			/*通信初始化  STM32 & NRF51822 */
	atkpInit();			/*传输协议初始化*/

	communicateInit();	/*和stablilizer通信的初始化*/
	
	pmInit();			/*电源管理初始化*/
	stabilizerInit();	/*电机 传感器 PID初始化*/

	watchdogInit(WATCHDOG_RESET_MS);	/*看门狗初始化*/
	
}


