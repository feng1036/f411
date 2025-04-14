#include "system.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ϵͳ��ʼ������	
 * ����ϵͳ�͵ײ�Ӳ����ʼ��
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

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
	}else
	{		
		while(1)		/*��ʼ������ ���Ϻ�Ƽ��1s����5��*/
		{
			if(cnt++ > 4)
			{
				cnt=0;
				delay_xms(1000);
			}
			ledFlashOne(LED_RED_R, 50, 50);		
		}
	}

	watchdogInit(WATCHDOG_RESET_MS);	/*���Ź���ʼ��*/
	
}

static bool systemTest(void)
{
	bool pass = true;
	// pass &= ledseqTest();
	// pass &= pmTest();
	// pass &= stabilizerTest();	
	// pass &= watchdogTest();
	return pass;
}

