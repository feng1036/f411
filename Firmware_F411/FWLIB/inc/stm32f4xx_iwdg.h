/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IWDG_H
#define __STM32F4xx_IWDG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#define IWDG_WriteAccess_Enable     ((uint16_t)0x5555)
#define IWDG_Prescaler_32           ((uint8_t)0x03)

/* Prescaler and Counter configuration functions ******************************/
void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);

/* IWDG activation function ***************************************************/
void IWDG_Enable(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IWDG_H */
