#ifndef __CONFIG_H
#define __CONFIG_H
#include "nvic.h"
#include "stdio.h"	/*printf ����*/

//#define CONFIG_PARAM_SIZE	(16*1024)

//#define CONFIG_PARAM_ADDR 	0x08078000 //(FLASH_BASE + BOOTLOADER_SIZE)	/*16K bootloader*/
#define FIRMWARE_START_ADDR 0x08000000 //(FLASH_BASE + BOOTLOADER_SIZE + CONFIG_PARAM_SIZE)	/*16K bootloader+ 16 ģ��eeprom*/

#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */

//#define MCU_ID_ADDRESS          0x1FFF7A10
//#define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22
#endif /* __CONFIG_H */
