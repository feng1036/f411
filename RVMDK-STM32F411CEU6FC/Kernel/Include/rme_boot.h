/******************************************************************************
Filename    : rme_boot.h
Author      : The RVM project generator.
Date        : 21/05/2025 22:33:52
License     : Unlicense; see COPYING for details.
Description : The boot-time initialization file header.
******************************************************************************/

/* Define ********************************************************************/
/* Capability table maximum capacity */
#define RME_MAIN_CPT_SIZE                               (128U)
/* Boot-time capability table */
#define RME_BOOT_CPT_OBJ                                (RME_A7M_CPT)

/* Vector endpoint capability tables */

/* Vector endpoints */

/* Receive endpoint capability tables - created by RVM later */
#define RME_BOOT_SIG_0                                  (15U)

/* Receive endpoints - created by RVM later */
#define RME_RCV_SLEEP_PRC_REALTIME                      (RME_CID(RME_BOOT_SIG_0,0U))

/* Code memory blocks */
#define RME_REALTIME_CODE_CODE1_BASE                    (0x8020000U)
#define RME_REALTIME_CODE_CODE1_SIZE                    (0x10000U)
#define RME_FLIGHT_CODE_CODE1_BASE                      (0x8030000U)
#define RME_FLIGHT_CODE_CODE1_SIZE                      (0x10000U)
#define RME_SENSOR_CODE_CODE1_BASE                      (0x8040000U)
#define RME_SENSOR_CODE_CODE1_SIZE                      (0x10000U)
#define RME_REMOTE_CODE_CODE1_BASE                      (0x8050000U)
#define RME_REMOTE_CODE_CODE1_SIZE                      (0x10000U)

/* Data memory blocks */
#define RME_REALTIME_DATA_DATA1_BASE                    (0x20008000U)
#define RME_REALTIME_DATA_DATA1_SIZE                    (0x4000U)
#define RME_FLIGHT_DATA_DATA1_BASE                      (0x2000C000U)
#define RME_FLIGHT_DATA_DATA1_SIZE                      (0x4000U)
#define RME_SENSOR_DATA_DATA1_BASE                      (0x20010000U)
#define RME_SENSOR_DATA_DATA1_SIZE                      (0x4000U)
#define RME_REMOTE_DATA_DATA1_BASE                      (0x20014000U)
#define RME_REMOTE_DATA_DATA1_SIZE                      (0x4000U)
#define RME_DATA_SHARED_SENSOR_BASE                     (0x20018000U)
#define RME_DATA_SHARED_SENSOR_SIZE                     (0x400U)
#define RME_DATA_SHARED_REMOTE_BASE                     (0x20018400U)
#define RME_DATA_SHARED_REMOTE_SIZE                     (0x400U)

/* Device memory blocks */
#define RME_DEVICE_SHARED_DEVICE_BASE                   (0x40000000U)
#define RME_DEVICE_SHARED_DEVICE_SIZE                   (0x20000000U)
/* End Define ****************************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/

