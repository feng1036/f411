
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_SPI_H
#define __STM32F4xx_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


typedef struct
{
  uint16_t SPI_Direction;           /*!< Specifies the SPI unidirectional or bidirectional data mode.
                                         This parameter can be a value of @ref SPI_data_direction */

  uint16_t SPI_Mode;                /*!< Specifies the SPI operating mode.
                                         This parameter can be a value of @ref SPI_mode */

  uint16_t SPI_DataSize;            /*!< Specifies the SPI data size.
                                         This parameter can be a value of @ref SPI_data_size */

  uint16_t SPI_CPOL;                /*!< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_Clock_Polarity */

  uint16_t SPI_CPHA;                /*!< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_Clock_Phase */

  uint16_t SPI_NSS;                 /*!< Specifies whether the NSS signal is managed by
                                         hardware (NSS pin) or by software using the SSI bit.
                                         This parameter can be a value of @ref SPI_Slave_Select_management */
 
  uint16_t SPI_BaudRatePrescaler;   /*!< Specifies the Baud Rate prescaler value which will be
                                         used to configure the transmit and receive SCK clock.
                                         This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                         @note The communication clock is derived from the master
                                               clock. The slave clock does not need to be set. */

  uint16_t SPI_FirstBit;            /*!< Specifies whether data transfers start from MSB or LSB bit.
                                         This parameter can be a value of @ref SPI_MSB_LSB_transmission */

  uint16_t SPI_CRCPolynomial;       /*!< Specifies the polynomial used for the CRC calculation. */
}SPI_InitTypeDef;


// typedef struct
// {

//   uint16_t I2S_Mode;         /*!< Specifies the I2S operating mode.
//                                   This parameter can be a value of @ref I2S_Mode */

//   uint16_t I2S_Standard;     /*!< Specifies the standard used for the I2S communication.
//                                   This parameter can be a value of @ref I2S_Standard */

//   uint16_t I2S_DataFormat;   /*!< Specifies the data format for the I2S communication.
//                                   This parameter can be a value of @ref I2S_Data_Format */

//   uint16_t I2S_MCLKOutput;   /*!< Specifies whether the I2S MCLK output is enabled or not.
//                                   This parameter can be a value of @ref I2S_MCLK_Output */

//   uint32_t I2S_AudioFreq;    /*!< Specifies the frequency selected for the I2S communication.
//                                   This parameter can be a value of @ref I2S_Audio_Frequency */

//   uint16_t I2S_CPOL;         /*!< Specifies the idle state of the I2S clock.
//                                   This parameter can be a value of @ref I2S_Clock_Polarity */
// }I2S_InitTypeDef;

#define SPI_Direction_2Lines_FullDuplex ((uint16_t)0x0000)

#define SPI_Mode_Master                 ((uint16_t)0x0104)
// #define SPI_DataSize_16b                ((uint16_t)0x0800)
#define SPI_DataSize_8b                 ((uint16_t)0x0000)

#define SPI_CPOL_Low                    ((uint16_t)0x0000)

#define SPI_CPHA_1Edge                  ((uint16_t)0x0000)

#define SPI_NSS_Soft                    ((uint16_t)0x0200)

#define SPI_BaudRatePrescaler_32        ((uint16_t)0x0020)

#define SPI_FirstBit_MSB                ((uint16_t)0x0000)
#define SPI_DeInit                   SPI_I2S_DeInit

void SPI_I2S_DeInit(SPI_TypeDef* SPIx);

void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);

#ifdef __cplusplus
}
#endif

#endif /*__STM32F4xx_SPI_H */
