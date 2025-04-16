#include "stm32f4xx_exti.h"

/**
  * @brief  Initializes the EXTI peripheral according to the specified
  *         parameters in the EXTI_InitStruct.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure
  *         that contains the configuration information for the EXTI peripheral.
  * @retval None
  */
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct)
{
  uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_EXTI_MODE(EXTI_InitStruct->EXTI_Mode));
  assert_param(IS_EXTI_TRIGGER(EXTI_InitStruct->EXTI_Trigger));
  assert_param(IS_EXTI_LINE(EXTI_InitStruct->EXTI_Line));  
  assert_param(IS_FUNCTIONAL_STATE(EXTI_InitStruct->EXTI_LineCmd));

  tmp = (uint32_t)EXTI_BASE;
     
  if (EXTI_InitStruct->EXTI_LineCmd != DISABLE)
  {
    /* Clear EXTI line configuration */
    EXTI->IMR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->EMR &= ~EXTI_InitStruct->EXTI_Line;
    
    tmp += EXTI_InitStruct->EXTI_Mode;

    *(__IO uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;

    /* Clear Rising Falling edge configuration */
    EXTI->RTSR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->FTSR &= ~EXTI_InitStruct->EXTI_Line;
    
    /* Select the trigger for the selected external interrupts */
    if (EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
    {
      /* Rising Falling edge */
      EXTI->RTSR |= EXTI_InitStruct->EXTI_Line;
      EXTI->FTSR |= EXTI_InitStruct->EXTI_Line;
    }
    else
    {
      tmp = (uint32_t)EXTI_BASE;
      tmp += EXTI_InitStruct->EXTI_Trigger;

      *(__IO uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;
    }
  }
  else
  {
    tmp += EXTI_InitStruct->EXTI_Mode;

    /* Disable the selected external lines */
    *(__IO uint32_t *) tmp &= ~EXTI_InitStruct->EXTI_Line;
  }
}

/**
  * @brief  Clears the EXTI's line pending flags.
  * @param  EXTI_Line: specifies the EXTI lines flags to clear.
  *          This parameter can be any combination of EXTI_Linex where x can be (0..22)
  * @retval None
  */
void EXTI_ClearFlag(uint32_t EXTI_Line)
{
  /* Check the parameters */
  assert_param(IS_EXTI_LINE(EXTI_Line));
  
  EXTI->PR = EXTI_Line;
}

/**
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *          This parameter can be EXTI_Linex where x can be(0..22)
  * @retval The new state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(EXTI_Line));
  
  if ((EXTI->PR & EXTI_Line) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
  
}

/**
  * @brief  Clears the EXTI's line pending bits.
  * @param  EXTI_Line: specifies the EXTI lines to clear.
  *          This parameter can be any combination of EXTI_Linex where x can be (0..22)
  * @retval None
  */
void EXTI_ClearITPendingBit(uint32_t EXTI_Line)
{
  /* Check the parameters */
  assert_param(IS_EXTI_LINE(EXTI_Line));
  
  EXTI->PR = EXTI_Line;
}
