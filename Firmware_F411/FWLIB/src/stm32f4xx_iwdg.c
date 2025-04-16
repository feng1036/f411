/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_iwdg.h"

/* KR register bit mask */
#define KR_KEY_RELOAD    ((uint16_t)0xAAAA)
#define KR_KEY_ENABLE    ((uint16_t)0xCCCC)

void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess)
{
  /* Check the parameters */
  assert_param(IS_IWDG_WRITE_ACCESS(IWDG_WriteAccess));
  IWDG->KR = IWDG_WriteAccess;
}

/**
  * @brief  Sets IWDG Prescaler value.
  * @param  IWDG_Prescaler: specifies the IWDG Prescaler value.
  *          This parameter can be one of the following values:
  *            @arg IWDG_Prescaler_4: IWDG prescaler set to 4
  *            @arg IWDG_Prescaler_8: IWDG prescaler set to 8
  *            @arg IWDG_Prescaler_16: IWDG prescaler set to 16
  *            @arg IWDG_Prescaler_32: IWDG prescaler set to 32
  *            @arg IWDG_Prescaler_64: IWDG prescaler set to 64
  *            @arg IWDG_Prescaler_128: IWDG prescaler set to 128
  *            @arg IWDG_Prescaler_256: IWDG prescaler set to 256
  * @retval None
  */
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler)
{
  /* Check the parameters */
  assert_param(IS_IWDG_PRESCALER(IWDG_Prescaler));
  IWDG->PR = IWDG_Prescaler;
}

/**
  * @brief  Sets IWDG Reload value.
  * @param  Reload: specifies the IWDG Reload value.
  *          This parameter must be a number between 0 and 0x0FFF.
  * @retval None
  */
void IWDG_SetReload(uint16_t Reload)
{
  /* Check the parameters */
  assert_param(IS_IWDG_RELOAD(Reload));
  IWDG->RLR = Reload;
}

/**
  * @brief  Reloads IWDG counter with value defined in the reload register
  *         (write access to IWDG_PR and IWDG_RLR registers disabled).
  * @param  None
  * @retval None
  */
void IWDG_ReloadCounter(void)
{
  IWDG->KR = KR_KEY_RELOAD;
}

void IWDG_Enable(void)
{
  IWDG->KR = KR_KEY_ENABLE;
}
