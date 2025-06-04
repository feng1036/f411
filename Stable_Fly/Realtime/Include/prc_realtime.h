/******************************************************************************
Filename    : prc_realtime.h
Author      : The RVM project generator.
Date        : 28/05/2025 22:00:00
License     : Unlicense; see COPYING for details.
Description : The user kernel object header.
******************************************************************************/

/* Define ********************************************************************/
#ifndef __PRC_REALTIME__
#define __PRC_REALTIME__

/* Process capability table frontier & size */
#define CPT_SIZE                                        (3U)
#define CPT_FRONT                                       (3U)

/* Ports */

/* Receive endpoints */
#define RCV_SLEEP                                       (1U)

/* Send endpoints */

/* Vector endpoints */

/* Kernel functions */
#define KFN_INT_LOCAL_MOD                               (2U)

/* Code memory blocks */
#define CODE_CODE1_BASE                                 (0x8020000U)
#define CODE_CODE1_SIZE                                 (0x10000U)

/* Data memory blocks */
#define DATA_DATA1_BASE                                 (0x20008000U)
#define DATA_DATA1_SIZE                                 (0x4000U)
#define DATA_SHARED_SENSOR_BASE                         (0x20018000U)
#define DATA_SHARED_SENSOR_SIZE                         (0x400U)
#define DATA_SHARED_REMOTE_BASE                         (0x20018400U)
#define DATA_SHARED_REMOTE_SIZE                         (0x400U)

/* Device memory blocks */
#define DEVICE_SHARED_DEVICE_BASE                       (0x40000000U)
#define DEVICE_SHARED_DEVICE_SIZE                       (0x20000000U)

/* Page table settings */
#define RVM_PGT_RAW_ENABLE                              (0U)
/* Total priority number */
#define RVM_PREEMPT_PRIO_NUM                            (32U)
/* Total VM priority number */
#define RVM_PREEMPT_VPRIO_NUM                           (32U)
/* The kernel memory allocation granularity order */
#define RVM_KOM_SLOT_ORDER                              (4U)

/* Virtual machine library disable */
#define RVM_VIRT_LIB_ENABLE                             (0U)

/* Debugging setting */
#define RVM_ASSERT_ENABLE                               (1U)
#define RVM_DBGLOG_ENABLE                               (1U)

/* Coprocessor option */
#define RVM_COP_NUM                                     (1U)
#define RVM_A7M_COP_FPV4_SP                             (1U)
#define RVM_A7M_COP_FPV5_SP                             (0U)
#define RVM_A7M_COP_FPV5_DP                             (0U)
#endif /* __PRC_REALTIME__ */
/* End Define ****************************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/


