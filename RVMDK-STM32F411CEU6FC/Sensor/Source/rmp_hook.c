/******************************************************************************
Filename    : rmp_hook_rvm.c
Author      : pry 
Date        : 08/09/2019
Licence     : The Unlicense; see LICENSE for details.
Description : The blank project hook file for RMP under RVM hypervisor.
******************************************************************************/

/* Include *******************************************************************/
#include "rvm.h"
#include "rmp.h"
/* End Include ***************************************************************/

/* Function:RMP_Init **********************************************************
Description : The init thread hook functions.
Input       : None.
Output      : None.
Return      : None.
******************************************************************************/
void RMP_Init_Hook(void)
{
	RMP_Clear(&Thd_1,sizeof(struct RMP_Thd));
	//RMP_Clear(&Fifo_1,sizeof(struct RMP_Fifo));
	//RMP_Clear(&Msgq_1,sizeof(struct RMP_Msgq));
	
	
	RVM_Hyp_Vct_Phys(31U,0U);
	RVM_Hyp_Vct_Phys(32U,1U);
	
	//send to Flight 
	RVM_Hyp_Evt_Add(11U);
	
	RVM_Virt_Vct_Reg(0U,I2C1_EV_IRQHandler);
	RVM_Virt_Vct_Reg(1U,I2C1_ER_IRQHandler);
	
	RVM_Virt_Vct_Lock();
	
	RMP_Thd_Crt(&Thd_1,(void*),(void*)0x1234U,Stack,1U,sizeof(Stack));

}

void RMP_Init_Idle(void)
{
    RVM_Hyp_Vct_Wait();
}
/* End Function:RMP_Init *****************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/

