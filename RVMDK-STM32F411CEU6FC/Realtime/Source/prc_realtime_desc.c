/******************************************************************************
Filename    : prc_realtime_desc.c
Author      : The RVM project generator.
Date        : 21/05/2025 22:33:52
License     : Unlicense; see COPYING for details.
Description : The process descriptor header file - do not edit!
              When using LTO, make sure this file is exempt from the LTO option,
              so that it be firmly linked to the head of the image!
******************************************************************************/

/* Include *******************************************************************/
#include "rvm.h"
/* End Include ***************************************************************/

/* Public Function ***********************************************************/
/* End Public Function *******************************************************/

/* Public Variable ***********************************************************/
const rvm_ptr_t RVM_Desc[4]=
{
RVM_MAGIC_NATIVE,
    0x2U,
    (rvm_ptr_t)__RVM_Entry,
    (rvm_ptr_t)__RVM_Stub,
};
/* End Public Variable *******************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/

