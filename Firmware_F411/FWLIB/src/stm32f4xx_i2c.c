#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#define CR1_CLEAR_MASK    ((uint16_t)0xFBF5)      /*<! I2C registers Masks */
#define FLAG_MASK         ((uint32_t)0x00FFFFFF)  /*<! I2C FLAG mask */

/**
  * @brief  Deinitialize the I2Cx peripheral registers to their default reset values.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @retval None
  */
void I2C_DeInit(I2C_TypeDef* I2Cx)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));

  if (I2Cx == I2C1)
  {
    /* Enable I2C1 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* Release I2C1 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);    
  }
  else if (I2Cx == I2C2)
  {
    /* Enable I2C2 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
    /* Release I2C2 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);      
  }
  else 
  {
    if (I2Cx == I2C3)
    {
      /* Enable I2C3 reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, ENABLE);
      /* Release I2C3 from reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, DISABLE);     
    }
  }
}

/**
  * @brief  Initializes the I2Cx peripheral according to the specified 
  *         parameters in the I2C_InitStruct.
  *           
  * @note   To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency 
  *         (I2C peripheral input clock) must be a multiple of 10 MHz.  
  *           
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_InitStruct: pointer to a I2C_InitTypeDef structure that contains 
  *         the configuration information for the specified I2C peripheral.
  * @retval None
  */
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct)
{
  uint16_t tmpreg = 0, freqrange = 0;
  uint16_t result = 0x04;
  uint32_t pclk1 = 8000000;
  RCC_ClocksTypeDef  rcc_clocks;
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_CLOCK_SPEED(I2C_InitStruct->I2C_ClockSpeed));
  assert_param(IS_I2C_MODE(I2C_InitStruct->I2C_Mode));
  assert_param(IS_I2C_DUTY_CYCLE(I2C_InitStruct->I2C_DutyCycle));
  assert_param(IS_I2C_OWN_ADDRESS1(I2C_InitStruct->I2C_OwnAddress1));
  assert_param(IS_I2C_ACK_STATE(I2C_InitStruct->I2C_Ack));
  assert_param(IS_I2C_ACKNOWLEDGE_ADDRESS(I2C_InitStruct->I2C_AcknowledgedAddress));

/*---------------------------- I2Cx CR2 Configuration ------------------------*/
  /* Get the I2Cx CR2 value */
  tmpreg = I2Cx->CR2;
  /* Clear frequency FREQ[5:0] bits */
  tmpreg &= (uint16_t)~((uint16_t)I2C_CR2_FREQ);
  /* Get pclk1 frequency value */
  RCC_GetClocksFreq(&rcc_clocks);
  pclk1 = rcc_clocks.PCLK1_Frequency;
  /* Set frequency bits depending on pclk1 value */
  freqrange = (uint16_t)(pclk1 / 1000000);
  tmpreg |= freqrange;
  /* Write to I2Cx CR2 */
  I2Cx->CR2 = tmpreg;

/*---------------------------- I2Cx CCR Configuration ------------------------*/
  /* Disable the selected I2C peripheral to configure TRISE */
  I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_PE);
  /* Reset tmpreg value */
  /* Clear F/S, DUTY and CCR[11:0] bits */
  tmpreg = 0;

  /* Configure speed in standard mode */
  if (I2C_InitStruct->I2C_ClockSpeed <= 100000)
  {
    /* Standard mode speed calculate */
    result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed << 1));
    /* Test if CCR value is under 0x4*/
    if (result < 0x04)
    {
      /* Set minimum allowed value */
      result = 0x04;  
    }
    /* Set speed value for standard mode */
    tmpreg |= result;	  
    /* Set Maximum Rise Time for standard mode */
    I2Cx->TRISE = freqrange + 1; 
  }
  /* Configure speed in fast mode */
  /* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
     input clock) must be a multiple of 10 MHz */
  else /*(I2C_InitStruct->I2C_ClockSpeed <= 400000)*/
  {
    if (I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_2)
    {
      /* Fast mode speed calculate: Tlow/Thigh = 2 */
      result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 3));
    }
    else /*I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_16_9*/
    {
      /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
      result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 25));
      /* Set DUTY bit */
      result |= I2C_DutyCycle_16_9;
    }

    /* Test if CCR value is under 0x1*/
    if ((result & I2C_CCR_CCR) == 0)
    {
      /* Set minimum allowed value */
      result |= (uint16_t)0x0001;  
    }
    /* Set speed value and set F/S bit for fast mode */
    tmpreg |= (uint16_t)(result | I2C_CCR_FS);
    /* Set Maximum Rise Time for fast mode */
    I2Cx->TRISE = (uint16_t)(((freqrange * (uint16_t)300) / (uint16_t)1000) + (uint16_t)1);  
  }

  /* Write to I2Cx CCR */
  I2Cx->CCR = tmpreg;
  /* Enable the selected I2C peripheral */
  I2Cx->CR1 |= I2C_CR1_PE;

/*---------------------------- I2Cx CR1 Configuration ------------------------*/
  /* Get the I2Cx CR1 value */
  tmpreg = I2Cx->CR1;
  /* Clear ACK, SMBTYPE and  SMBUS bits */
  tmpreg &= CR1_CLEAR_MASK;
  /* Configure I2Cx: mode and acknowledgement */
  /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
  /* Set ACK bit according to I2C_Ack value */
  tmpreg |= (uint16_t)((uint32_t)I2C_InitStruct->I2C_Mode | I2C_InitStruct->I2C_Ack);
  /* Write to I2Cx CR1 */
  I2Cx->CR1 = tmpreg;

/*---------------------------- I2Cx OAR1 Configuration -----------------------*/
  /* Set I2Cx Own Address1 and acknowledged address */
  I2Cx->OAR1 = (I2C_InitStruct->I2C_AcknowledgedAddress | I2C_InitStruct->I2C_OwnAddress1);
}

/**
  * @brief  Transmits the address byte to select the slave device.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Address: specifies the slave address which will be transmitted
  * @param  I2C_Direction: specifies whether the I2C device will be a Transmitter
  *         or a Receiver. 
  *          This parameter can be one of the following values
  *            @arg I2C_Direction_Transmitter: Transmitter mode
  *            @arg I2C_Direction_Receiver: Receiver mode
  * @retval None.
  */
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_DIRECTION(I2C_Direction));
  /* Test on the direction to set/reset the read/write bit */
  if (I2C_Direction != I2C_Direction_Transmitter)
  {
    /* Set the address bit0 for read */
    Address |= I2C_OAR1_ADD0;
  }
  else
  {
    /* Reset the address bit0 for write */
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
  }
  /* Send the address */
  I2Cx->DR = Address;
}

/**
  * @brief  Enables or disables the specified I2C acknowledge feature.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C Acknowledgement.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the acknowledgement */
    I2Cx->CR1 |= I2C_CR1_ACK;
  }
  else
  {
    /* Disable the acknowledgement */
    I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
  }
}

/**
  * @brief  Sends a data byte through the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Data: Byte to be transmitted..
  * @retval None
  */
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  /* Write in the DR register the data to be sent */
  I2Cx->DR = Data;
}

/**
  * @brief  Returns the most recent received data by the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @retval The value of the received data.
  */
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  /* Return the data in the DR register */
  return (uint8_t)I2Cx->DR;
}

/**
  * @brief  Enables or disables the specified I2C DMA requests.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C DMA transfer.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected I2C DMA requests */
    I2Cx->CR2 |= I2C_CR2_DMAEN;
  }
  else
  {
    /* Disable the selected I2C DMA requests */
    I2Cx->CR2 &= (uint16_t)~((uint16_t)I2C_CR2_DMAEN);
  }
}

/**
  * @brief  Specifies that the next DMA transfer is the last one.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C DMA last transfer.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Next DMA transfer is the last transfer */
    I2Cx->CR2 |= I2C_CR2_LAST;
  }
  else
  {
    /* Next DMA transfer is not the last transfer */
    I2Cx->CR2 &= (uint16_t)~((uint16_t)I2C_CR2_LAST);
  }
}

/**
  * @brief  Enables or disables the specified I2C interrupts.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_IT: specifies the I2C interrupts sources to be enabled or disabled. 
  *          This parameter can be any combination of the following values:
  *            @arg I2C_IT_BUF: Buffer interrupt mask
  *            @arg I2C_IT_EVT: Event interrupt mask
  *            @arg I2C_IT_ERR: Error interrupt mask
  * @param  NewState: new state of the specified I2C interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_I2C_CONFIG_IT(I2C_IT));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected I2C interrupts */
    I2Cx->CR2 |= I2C_IT;
  }
  else
  {
    /* Disable the selected I2C interrupts */
    I2Cx->CR2 &= (uint16_t)~I2C_IT;
  }
}

/**
  * @brief  Checks whether the specified I2C flag is set or not.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to check. 
  *          This parameter can be one of the following values:
  *            @arg I2C_FLAG_DUALF: Dual flag (Slave mode)
  *            @arg I2C_FLAG_SMBHOST: SMBus host header (Slave mode)
  *            @arg I2C_FLAG_SMBDEFAULT: SMBus default header (Slave mode)
  *            @arg I2C_FLAG_GENCALL: General call header flag (Slave mode)
  *            @arg I2C_FLAG_TRA: Transmitter/Receiver flag
  *            @arg I2C_FLAG_BUSY: Bus busy flag
  *            @arg I2C_FLAG_MSL: Master/Slave flag
  *            @arg I2C_FLAG_SMBALERT: SMBus Alert flag
  *            @arg I2C_FLAG_TIMEOUT: Timeout or Tlow error flag
  *            @arg I2C_FLAG_PECERR: PEC error in reception flag
  *            @arg I2C_FLAG_OVR: Overrun/Underrun flag (Slave mode)
  *            @arg I2C_FLAG_AF: Acknowledge failure flag
  *            @arg I2C_FLAG_ARLO: Arbitration lost flag (Master mode)
  *            @arg I2C_FLAG_BERR: Bus error flag
  *            @arg I2C_FLAG_TXE: Data register empty flag (Transmitter)
  *            @arg I2C_FLAG_RXNE: Data register not empty (Receiver) flag
  *            @arg I2C_FLAG_STOPF: Stop detection flag (Slave mode)
  *            @arg I2C_FLAG_ADD10: 10-bit header sent flag (Master mode)
  *            @arg I2C_FLAG_BTF: Byte transfer finished flag
  *            @arg I2C_FLAG_ADDR: Address sent flag (Master mode) "ADSL"
  *                                Address matched flag (Slave mode)"ENDAD"
  *            @arg I2C_FLAG_SB: Start bit flag (Master mode)
  * @retval The new state of I2C_FLAG (SET or RESET).
  */
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG)
{
  FlagStatus bitstatus = RESET;
  __IO uint32_t i2creg = 0, i2cxbase = 0;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_GET_FLAG(I2C_FLAG));

  /* Get the I2Cx peripheral base address */
  i2cxbase = (uint32_t)I2Cx;
  
  /* Read flag register index */
  i2creg = I2C_FLAG >> 28;
  
  /* Get bit[23:0] of the flag */
  I2C_FLAG &= FLAG_MASK;
  
  if(i2creg != 0)
  {
    /* Get the I2Cx SR1 register address */
    i2cxbase += 0x14;
  }
  else
  {
    /* Flag in I2Cx SR2 Register */
    I2C_FLAG = (uint32_t)(I2C_FLAG >> 16);
    /* Get the I2Cx SR2 register address */
    i2cxbase += 0x18;
  }
  
  if(((*(__IO uint32_t *)i2cxbase) & I2C_FLAG) != (uint32_t)RESET)
  {
    /* I2C_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* I2C_FLAG is reset */
    bitstatus = RESET;
  }
  
  /* Return the I2C_FLAG status */
  return  bitstatus;
}

/**
  * @brief  Clears the I2Cx's pending flags.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to clear. 
  *          This parameter can be any combination of the following values:
  *            @arg I2C_FLAG_SMBALERT: SMBus Alert flag
  *            @arg I2C_FLAG_TIMEOUT: Timeout or Tlow error flag
  *            @arg I2C_FLAG_PECERR: PEC error in reception flag
  *            @arg I2C_FLAG_OVR: Overrun/Underrun flag (Slave mode)
  *            @arg I2C_FLAG_AF: Acknowledge failure flag
  *            @arg I2C_FLAG_ARLO: Arbitration lost flag (Master mode)
  *            @arg I2C_FLAG_BERR: Bus error flag
  *   
  * @note   STOPF (STOP detection) is cleared by software sequence: a read operation 
  *          to I2C_SR1 register (I2C_GetFlagStatus()) followed by a write operation 
  *          to I2C_CR1 register (I2C_Cmd() to re-enable the I2C peripheral).
  * @note   ADD10 (10-bit header sent) is cleared by software sequence: a read 
  *          operation to I2C_SR1 (I2C_GetFlagStatus()) followed by writing the 
  *          second byte of the address in DR register.
  * @note   BTF (Byte Transfer Finished) is cleared by software sequence: a read 
  *          operation to I2C_SR1 register (I2C_GetFlagStatus()) followed by a 
  *          read/write to I2C_DR register (I2C_SendData()).
  * @note   ADDR (Address sent) is cleared by software sequence: a read operation to 
  *          I2C_SR1 register (I2C_GetFlagStatus()) followed by a read operation to 
  *          I2C_SR2 register ((void)(I2Cx->SR2)).
  * @note   SB (Start Bit) is cleared software sequence: a read operation to I2C_SR1
  *          register (I2C_GetFlagStatus()) followed by a write operation to I2C_DR
  *          register (I2C_SendData()).
  *  
  * @retval None
  */
void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG)
{
  uint32_t flagpos = 0;
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_CLEAR_FLAG(I2C_FLAG));
  /* Get the I2C flag position */
  flagpos = I2C_FLAG & FLAG_MASK;
  /* Clear the selected I2C flag */
  I2Cx->SR1 = (uint16_t)~flagpos;
}
