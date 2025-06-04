/******************************************************************************
Filename    : rvm_boot.h
Author      : The RVM project generator.
Date        : 28/05/2025 22:00:00
License     : Unlicense; see COPYING for details.
Description : The boot-time initialization file header.
******************************************************************************/

/* Define ********************************************************************/
/* Vector endpoint capability tables */

/* Vector endpoints */

/* 2 threads and 1 endpoint for RVM */

/* Virtual machine endpoint capability tables */
#define RVM_MAIN_VEP_0                                  (10U)

/* Process capability table capability tables */
#define RVM_MAIN_CPT_0                                  (11U)

/* Process capability tables */
#define RVM_CPT_REALTIME                                (RVM_CID(RVM_MAIN_CPT_0, 0U))
#define RVM_CPT_FLIGHT                                  (RVM_CID(RVM_MAIN_CPT_0, 1U))
#define RVM_CPT_SENSOR                                  (RVM_CID(RVM_MAIN_CPT_0, 2U))
#define RVM_CPT_REMOTE                                  (RVM_CID(RVM_MAIN_CPT_0, 3U))

/* Process page table capability tables */
#define RVM_MAIN_PGT_0                                  (12U)

/* Process page tables */
#define RVM_PGT_REALTIME_0                              (RVM_CID(RVM_MAIN_PGT_0, 0U))
#define RVM_PGT_REALTIME_1                              (RVM_CID(RVM_MAIN_PGT_0, 1U))
#define RVM_PGT_REALTIME_2                              (RVM_CID(RVM_MAIN_PGT_0, 2U))
#define RVM_PGT_REALTIME_3                              (RVM_CID(RVM_MAIN_PGT_0, 3U))
#define RVM_PGT_FLIGHT_0                                (RVM_CID(RVM_MAIN_PGT_0, 4U))
#define RVM_PGT_FLIGHT_1                                (RVM_CID(RVM_MAIN_PGT_0, 5U))
#define RVM_PGT_FLIGHT_2                                (RVM_CID(RVM_MAIN_PGT_0, 6U))
#define RVM_PGT_FLIGHT_3                                (RVM_CID(RVM_MAIN_PGT_0, 7U))
#define RVM_PGT_SENSOR_0                                (RVM_CID(RVM_MAIN_PGT_0, 8U))
#define RVM_PGT_SENSOR_1                                (RVM_CID(RVM_MAIN_PGT_0, 9U))
#define RVM_PGT_SENSOR_2                                (RVM_CID(RVM_MAIN_PGT_0, 10U))
#define RVM_PGT_SENSOR_3                                (RVM_CID(RVM_MAIN_PGT_0, 11U))
#define RVM_PGT_REMOTE_0                                (RVM_CID(RVM_MAIN_PGT_0, 12U))
#define RVM_PGT_REMOTE_1                                (RVM_CID(RVM_MAIN_PGT_0, 13U))
#define RVM_PGT_REMOTE_2                                (RVM_CID(RVM_MAIN_PGT_0, 14U))
#define RVM_PGT_REMOTE_3                                (RVM_CID(RVM_MAIN_PGT_0, 15U))

/* Process capability tables */
#define RVM_MAIN_PRC_0                                  (13U)

/* Processes */
#define RVM_PRC_REALTIME                                (RVM_CID(RVM_MAIN_PRC_0, 0U))
#define RVM_PRC_FLIGHT                                  (RVM_CID(RVM_MAIN_PRC_0, 1U))
#define RVM_PRC_SENSOR                                  (RVM_CID(RVM_MAIN_PRC_0, 2U))
#define RVM_PRC_REMOTE                                  (RVM_CID(RVM_MAIN_PRC_0, 3U))

/* Thread capability tables */
#define RVM_MAIN_THD_0                                  (14U)

/* Threads */
#define RVM_THD_STARTUP_PRC_REALTIME                    (RVM_CID(RVM_MAIN_THD_0, 0U))
#define RVM_THD_VCT_PRC_FLIGHT                          (RVM_CID(RVM_MAIN_THD_0, 1U))
#define RVM_THD_USR_PRC_FLIGHT                          (RVM_CID(RVM_MAIN_THD_0, 2U))
#define RVM_THD_VCT_PRC_SENSOR                          (RVM_CID(RVM_MAIN_THD_0, 3U))
#define RVM_THD_USR_PRC_SENSOR                          (RVM_CID(RVM_MAIN_THD_0, 4U))
#define RVM_THD_VCT_PRC_REMOTE                          (RVM_CID(RVM_MAIN_THD_0, 5U))
#define RVM_THD_USR_PRC_REMOTE                          (RVM_CID(RVM_MAIN_THD_0, 6U))

/* Invocation capability tables */

/* Invocations */

/* Receive endpoint capability tables */
#define RVM_MAIN_RCV_0                                  (15U)

/* Receive endpoints */
#define RVM_RCV_SLEEP_PRC_REALTIME                      (RVM_CID(RVM_MAIN_RCV_0, 0U))

/* VM number */
#define RVM_VIRT_NUM                                    (3U)

/* Virtual endpoint frontiers & number */
#define RVM_BOOT_VEP_BEFORE                             (0x410U)
#define RVM_BOOT_VEP_AFTER                              (0x470U)
#define RVM_BOOT_VEP_MAIN_NUM                           (1U)
#define RVM_BOOT_VEP_CRT_NUM                            (3U)
#define RVM_BOOT_VCAP_INIT_NUM                          (3U)

/* Cpt frontiers & number */
#define RVM_BOOT_INIT_CPT_BEFORE                        (0x470U)
#define RVM_BOOT_INIT_CPT_AFTER                         (0x610U)
#define RVM_BOOT_INIT_CPT_MAIN_NUM                      (1U)
#define RVM_BOOT_INIT_CPT_CRT_NUM                       (4U)
#define RVM_BOOT_INIT_CPT_INIT_NUM                      (1U)
#define RVM_BOOT_INIT_CPT_KFN_NUM                       (2U)

/* Pgt frontiers & number */
#define RVM_BOOT_INIT_PGT_BEFORE                        (0x610U)
#define RVM_BOOT_INIT_PGT_AFTER                         (0xBB0U)
#define RVM_BOOT_INIT_PGT_MAIN_NUM                      (1U)
#define RVM_BOOT_INIT_PGT_CRT_NUM                       (16U)
#define RVM_BOOT_INIT_PGT_CON_NUM                       (12U)
#define RVM_BOOT_INIT_PGT_ADD_NUM                       (20U)

/* Process frontiers & number */
#define RVM_BOOT_PRC_BEFORE                             (0xBB0U)
#define RVM_BOOT_PRC_AFTER                              (0xC30U)
#define RVM_BOOT_PRC_MAIN_NUM                           (1U)
#define RVM_BOOT_PRC_CRT_NUM                            (4U)

/* Thread frontiers & number */
#define RVM_BOOT_THD_BEFORE                             (0xC30U)
#define RVM_BOOT_THD_AFTER                              (0x1170U)
#define RVM_BOOT_THD_MAIN_NUM                           (1U)
#define RVM_BOOT_THD_CRT_NUM                            (7U)
#define RVM_BOOT_THD_INIT_NUM                           (7U)

/* Invocation frontiers & number */
#define RVM_BOOT_INV_BEFORE                             (0x1170U)
#define RVM_BOOT_INV_AFTER                              (0x1170U)
#define RVM_BOOT_INV_MAIN_NUM                           (0U)
#define RVM_BOOT_INV_CRT_NUM                            (0U)
#define RVM_BOOT_INV_INIT_NUM                           (0U)

/* Receive endpoint frontiers & number */
#define RVM_BOOT_RCV_BEFORE                             (0x1170U)
#define RVM_BOOT_RCV_AFTER                              (0x1190U)
#define RVM_BOOT_RCV_MAIN_NUM                           (1U)
#define RVM_BOOT_RCV_CRT_NUM                            (1U)

/* Code memory blocks */
#define RVM_REALTIME_CODE_CODE1_BASE                    (0x8020000U)
#define RVM_REALTIME_CODE_CODE1_SIZE                    (0x10000U)
#define RVM_FLIGHT_CODE_CODE1_BASE                      (0x8030000U)
#define RVM_FLIGHT_CODE_CODE1_SIZE                      (0x10000U)
#define RVM_SENSOR_CODE_CODE1_BASE                      (0x8040000U)
#define RVM_SENSOR_CODE_CODE1_SIZE                      (0x10000U)
#define RVM_REMOTE_CODE_CODE1_BASE                      (0x8050000U)
#define RVM_REMOTE_CODE_CODE1_SIZE                      (0x10000U)

/* Data memory blocks */
#define RVM_REALTIME_DATA_DATA1_BASE                    (0x20008000U)
#define RVM_REALTIME_DATA_DATA1_SIZE                    (0x4000U)
#define RVM_FLIGHT_DATA_DATA1_BASE                      (0x2000C000U)
#define RVM_FLIGHT_DATA_DATA1_SIZE                      (0x4000U)
#define RVM_SENSOR_DATA_DATA1_BASE                      (0x20010000U)
#define RVM_SENSOR_DATA_DATA1_SIZE                      (0x4000U)
#define RVM_REMOTE_DATA_DATA1_BASE                      (0x20014000U)
#define RVM_REMOTE_DATA_DATA1_SIZE                      (0x4000U)
#define RVM_DATA_SHARED_SENSOR_BASE                     (0x20018000U)
#define RVM_DATA_SHARED_SENSOR_SIZE                     (0x400U)
#define RVM_DATA_SHARED_REMOTE_BASE                     (0x20018400U)
#define RVM_DATA_SHARED_REMOTE_SIZE                     (0x400U)

/* Device memory blocks */
#define RVM_DEVICE_SHARED_DEVICE_BASE                   (0x40000000U)
#define RVM_DEVICE_SHARED_DEVICE_SIZE                   (0x20000000U)
/* End Define ****************************************************************/

/* End Of File ***************************************************************/

/* Copyright (C) Evo-Devo Instrum. All rights reserved ***********************/

