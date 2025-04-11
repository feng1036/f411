#include "system.h"


/*�ײ�Ӳ����ʼ��*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*�ж����ó�ʼ��*/
	extiInit();			/*�ⲿ�жϳ�ʼ��*/	
	delay_init(96);		/*delay��ʼ��*/
	ledInit();			/*led��ʼ��*/
	ledseqInit();		/*led�����г�ʼ��*/
	
	commInit();			/*ͨ�ų�ʼ��  STM32 & NRF51822 */
	atkpInit();			/*����Э���ʼ��*/

	communicateInit();	/*��stablilizerͨ�ŵĳ�ʼ��*/
	
	pmInit();			/*��Դ�����ʼ��*/
	stabilizerInit();	/*��� ������ PID��ʼ��*/

	watchdogInit(WATCHDOG_RESET_MS);	/*���Ź���ʼ��*/
	
}


