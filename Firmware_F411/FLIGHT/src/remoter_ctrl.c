#include "string.h"
#include <math.h>
#include "sensfusion6.h"
#include "remoter_ctrl.h"
#include "config_param.h"
#include "commander.h"
#include "radiolink.h"
#include "sensors.h"
#include "stabilizer.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

static ctrlVal_t remoterCtrl;/*���͵�commander��̬��������*/


/*ң�����ݽ��մ���*/
void remoterCtrlProcess(atkp_t* pk)
{	
	if(pk->data[0] == REMOTER_CMD)
	{
		switch(pk->data[1])
		{
			case CMD_FLIGHT_LAND:
				if(getCommanderKeyFlight() != true)
				{
					setCommanderKeyFlight(true);
					setCommanderKeyland(false);
				}
				else
				{
					setCommanderKeyFlight(false);
					setCommanderKeyland(true);
				}
				break;

			case CMD_EMER_STOP:
				setCommanderKeyFlight(false);
				setCommanderKeyland(false);
				break;
		}
	}
	else if(pk->data[0] == REMOTER_DATA)
	{
		remoterData_t remoterData = *(remoterData_t*)(pk->data+1);
		
		remoterCtrl.roll = remoterData.roll;
		remoterCtrl.pitch = remoterData.pitch;
		remoterCtrl.yaw = remoterData.yaw;
		remoterCtrl.thrust = remoterData.thrust * 655.35f;
		remoterCtrl.trimPitch = remoterData.trimPitch;
		remoterCtrl.trimRoll = remoterData.trimRoll;
		
		setCommanderCtrlMode(remoterData.ctrlMode);
		setCommanderFlightmode(remoterData.flightMode);
		flightCtrldataCache(ATK_REMOTER, remoterCtrl);
	}
}

