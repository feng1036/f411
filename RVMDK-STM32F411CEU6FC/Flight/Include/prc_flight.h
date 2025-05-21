/******************************************************************************
Filename    : prc_flight.h
Author      : The RVM project generator.
Date        : 21/05/2025 22:33:52
License     : Unlicense; see COPYING for details.
Description : The user kernel object header.
******************************************************************************/

/* Define ********************************************************************/
#ifndef __PRC_FLIGHT__
#define __PRC_FLIGHT__

/* Process capability table frontier & size */
#define CPT_SIZE                                        (2U)
#define CPT_FRONT                                       (2U)

/* Ports */

/* Receive endpoints */

/* Send endpoints */

/* Vector endpoints */

/* Kernel functions */

/* Code memory blocks */
#define CODE_CODE1_BASE                                 (0x8030000U)
#define CODE_CODE1_SIZE                                 (0x10000U)

/* Data memory blocks */
#define DATA_DATA1_BASE                                 (0x2000C000U)
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

/* Virtual machine library enable */
#define RVM_VIRT_LIB_ENABLE                             (1U)
/* Virtual vector total number */
#define RVM_VIRT_VCT_NUM                                (2U)
/* State block base address & size */
#define RVM_VIRT_STATE_BASE                             (0x2000F9C0U)
#define RVM_VIRT_STATE_SIZE                             (0x40U)
/* Virtual register base address & size */
#define RVM_VIRT_REG_BASE                               (0x2000F950U)
#define RVM_VIRT_REG_SIZE                               (0x70U)

/* Debugging setting */
#define RVM_ASSERT_ENABLE                               (1U)
#define RVM_DBGLOG_ENABLE                               (1U)

/* Coprocessor option */
#define RVM_COP_NUM                                     (1U)
#define RVM_A7M_COP_FPV4_SP                             (1U)
#define RVM_A7M_COP_FPV5_SP                             (0U)
#define RVM_A7M_COP_FPV5_DP                             (0U)
#endif /* __PRC_FLIGHT__ */
/* End Define ****************************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/


