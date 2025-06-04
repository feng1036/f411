/******************************************************************************
Filename    : prc_realtime.c
Author      : The RVM project generator.
Date        : 28/05/2025 22:00:00
License     : Unlicense; see COPYING for details.
Description : The main user stub file.
******************************************************************************/

/* Include *******************************************************************/
#include "rvm.h"
/* End Include ***************************************************************/

/* Public Variable ***********************************************************/
RVM_EXTERN const rvm_ptr_t RVM_Desc[4];
/* End Public Variable *******************************************************/

/* Private Function **********************************************************/
RVM_EXTERN rvm_ret_t Thd_Startup(rvm_ret_t Param);
RVM_EXTERN void _RVM_Stub(void);
void RVM_Putchar(char Char);
/* End Private Function ******************************************************/

/* Begin Function:main ********************************************************
Description : The function body for thread.
Input       : None.
Output      : None.
Return      : rvm_ret_t - Should never return.
******************************************************************************/
rvm_ret_t main(void)
{
    /* Check header validity */
    RVM_ASSERT(RVM_Desc[0]==RVM_MAGIC_NATIVE);
    RVM_ASSERT(RVM_Desc[1]==2U);

    /* Call the first (highest-priority) thread */
    Thd_Startup(1234U);
}
/* End Function:main *********************************************************/

/* Begin Function:RVM_Putchar *************************************************
Description : The character printing function for debugging.
Input       : char Char - The character to print to console.
Output      : None.
Return      : None.
******************************************************************************/
#if(RVM_DBGLOG_ENABLE!=0U)
void RVM_Putchar(char Char)
{
    /* Add your own code */
}
#endif
/* End Function:RVM_Putchar **************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/

