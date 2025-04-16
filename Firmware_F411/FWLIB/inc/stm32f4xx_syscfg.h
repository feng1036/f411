/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_SYSCFG_H
#define __STM32F4xx_SYSCFG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define EXTI_PortSourceGPIOA       ((uint8_t)0x00)
#define EXTI_PinSource4            ((uint8_t)0x04)

void       SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);

#ifdef __cplusplus
}
#endif

#endif /*__STM32F4xx_SYSCFG_H */
