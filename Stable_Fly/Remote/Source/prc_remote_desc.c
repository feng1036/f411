/******************************************************************************
Filename    : prc_remote_desc.c
Author      : The RVM project generator.
Date        : 28/05/2025 22:00:00
License     : Unlicense; see COPYING for details.
Description : The process descriptor header file - do not edit!
              When using LTO, make sure this file is exempt from the LTO option,
              so that it be firmly linked to the head of the image!
******************************************************************************/

/* Include *******************************************************************/
#include "rvm.h"
/* End Include ***************************************************************/

/* Public Function ***********************************************************/
RVM_EXTERN rvm_ret_t Thd_Vct(rvm_ret_t Param);
/* End Public Function *******************************************************/

/* Public Variable ***********************************************************/
const rvm_ptr_t RVM_Desc[5]=
{
RVM_MAGIC_VIRTUAL,
    0x3U,
    (rvm_ptr_t)Thd_Vct,
    (rvm_ptr_t)__RVM_Entry,
    (rvm_ptr_t)__RVM_Stub,
};
/* End Public Variable *******************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/

