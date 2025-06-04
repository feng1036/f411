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
volatile struct RMP_Thd Thd_F;
void stabilizerTask(void);
void Int_Init(void);
rmp_ptr_t Stack_F[1024];

void Contact_Sensor(void);
void Contact_Remote(void);

/* Function:RMP_Init **********************************************************
Description : The init thread hook functions.
Input       : None.
Output      : None.
Return      : None.
******************************************************************************/
void RMP_Init_Hook(void)
{
		/* Clean up the structures */
    RMP_Clear(&Thd_F,sizeof(struct RMP_Thd));
	
		RVM_Hyp_Evt_Add(11U);
		RVM_Hyp_Evt_Add(22U);
	
		RVM_Hyp_Vct_Evt(11U,0U);
		RVM_Hyp_Vct_Evt(22U,1U);
	
		RVM_Virt_Vct_Reg(0U,Contact_Sensor);
    RVM_Virt_Vct_Reg(1U,Contact_Remote);
	
		Int_Init();
	
    /* Create kernel objects */
    RMP_Thd_Crt(&Thd_F,(void*)stabilizerTask,(void*)0x6666U,Stack_F,sizeof(Stack_F),1U,100U);
}

void RMP_Init_Idle(void)
{
    RVM_Hyp_Vct_Wait();
}
/* End Function:RMP_Init *****************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/

