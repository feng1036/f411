
// /* SPI registers Masks */
#define CR1_CLEAR_MASK            ((uint16_t)0x3040)

void SPI_I2S_DeInit(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));

  if (SPIx == SPI1)
  {
    /* Enable SPI1 reset state */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
    /* Release SPI1 from reset state */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
  }
  else if (SPIx == SPI2)
  {
    /* Enable SPI2 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);
    /* Release SPI2 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);
  }
  else if (SPIx == SPI3)
  {
    /* Enable SPI3 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
    /* Release SPI3 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
  }
  else if (SPIx == SPI4)
  {
    /* Enable SPI4 reset state */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI4, ENABLE);
    /* Release SPI4 from reset state */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI4, DISABLE);
  }
  else if (SPIx == SPI5)
  {
    /* Enable SPI5 reset state */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5, ENABLE);
    /* Release SPI5 from reset state */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5, DISABLE);
  }
  else 
  {
    if (SPIx == SPI6)
    {
      /* Enable SPI6 reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI6, ENABLE);
      /* Release SPI6 from reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI6, DISABLE);
    }
  }
}

/**
  * @brief  Initializes the SPIx peripheral according to the specified 
  *         parameters in the SPI_InitStruct.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure that
  *         contains the configuration information for the specified SPI peripheral.
  * @retval None
  */
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct)
{
  uint16_t tmpreg = 0;
  
  /* check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  /* Check the SPI parameters */
  assert_param(IS_SPI_DIRECTION_MODE(SPI_InitStruct->SPI_Direction));
  assert_param(IS_SPI_MODE(SPI_InitStruct->SPI_Mode));
  assert_param(IS_SPI_DATASIZE(SPI_InitStruct->SPI_DataSize));
  assert_param(IS_SPI_CPOL(SPI_InitStruct->SPI_CPOL));
  assert_param(IS_SPI_CPHA(SPI_InitStruct->SPI_CPHA));
  assert_param(IS_SPI_NSS(SPI_InitStruct->SPI_NSS));
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_InitStruct->SPI_BaudRatePrescaler));
  assert_param(IS_SPI_FIRST_BIT(SPI_InitStruct->SPI_FirstBit));
  assert_param(IS_SPI_CRC_POLYNOMIAL(SPI_InitStruct->SPI_CRCPolynomial));

/*---------------------------- SPIx CR1 Configuration ------------------------*/
  /* Get the SPIx CR1 value */
  tmpreg = SPIx->CR1;
  /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, MSTR, CPOL and CPHA bits */
  tmpreg &= CR1_CLEAR_MASK;
  /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
     master/salve mode, CPOL and CPHA */
  /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
  /* Set SSM, SSI and MSTR bits according to SPI_Mode and SPI_NSS values */
  /* Set LSBFirst bit according to SPI_FirstBit value */
  /* Set BR bits according to SPI_BaudRatePrescaler value */
  /* Set CPOL bit according to SPI_CPOL value */
  /* Set CPHA bit according to SPI_CPHA value */
  tmpreg |= (uint16_t)((uint32_t)SPI_InitStruct->SPI_Direction | SPI_InitStruct->SPI_Mode |
                  SPI_InitStruct->SPI_DataSize | SPI_InitStruct->SPI_CPOL |  
                  SPI_InitStruct->SPI_CPHA | SPI_InitStruct->SPI_NSS |  
                  SPI_InitStruct->SPI_BaudRatePrescaler | SPI_InitStruct->SPI_FirstBit);
  /* Write to SPIx CR1 */
  SPIx->CR1 = tmpreg;

  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  SPIx->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
/*---------------------------- SPIx CRCPOLY Configuration --------------------*/
  /* Write to SPIx CRCPOLY */
  SPIx->CRCPR = SPI_InitStruct->SPI_CRCPolynomial;
}
