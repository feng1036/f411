#include "system.h"


static bool systemTest(void);

/*�ײ�Ӳ����ʼ��*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*�ж����ó�ʼ��*/
	extiInit();			/*�ⲿ�жϳ�ʼ��*/	
	delay_init(96);		/*delay��ʼ��*/
	ledInit();			/*led��ʼ��*/
	ledseqInit();		/*led�����г�ʼ��*/
	communicateInit();	/*��stablilizerͨ�ŵĳ�ʼ��*/
	uartslkInit();		/*���ڳ�ʼ��*/
	pmInit();			/*��Դ�����ʼ��*/
	stabilizerInit();	/*��� ������ PID��ʼ��*/
	if(systemTest() == true)
	{	
		while(cnt++ < 5)	/*��ʼ��ͨ�� �����̵ƿ���5��*/
		{
			ledFlashOne(LED_GREEN_L, 50, 50);
		}			
	}
	watchdogInit(WATCHDOG_RESET_MS);	/*���Ź���ʼ��*/
}

static bool systemTest(void)
{
	bool pass = true;
	return pass;
}

