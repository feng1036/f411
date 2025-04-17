/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_DMA_H
#define __STM32F4xx_DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"

typedef struct
{
  uint32_t DMA_Channel;            /*!< Specifies the channel used for the specified stream. 
                                        This parameter can be a value of @ref DMA_channel */
 
  uint32_t DMA_PeripheralBaseAddr; /*!< Specifies the peripheral base address for DMAy Streamx. */

  uint32_t DMA_Memory0BaseAddr;    /*!< Specifies the memory 0 base address for DMAy Streamx. 
                                        This memory is the default memory used when double buffer mode is
                                        not enabled. */

  uint32_t DMA_DIR;                /*!< Specifies if the data will be transferred from memory to peripheral, 
                                        from memory to memory or from peripheral to memory.
                                        This parameter can be a value of @ref DMA_data_transfer_direction */

  uint32_t DMA_BufferSize;         /*!< Specifies the buffer size, in data unit, of the specified Stream. 
                                        The data unit is equal to the configuration set in DMA_PeripheralDataSize
                                        or DMA_MemoryDataSize members depending in the transfer direction. */

  uint32_t DMA_PeripheralInc;      /*!< Specifies whether the Peripheral address register should be incremented or not.
                                        This parameter can be a value of @ref DMA_peripheral_incremented_mode */

  uint32_t DMA_MemoryInc;          /*!< Specifies whether the memory address register should be incremented or not.
                                        This parameter can be a value of @ref DMA_memory_incremented_mode */

  uint32_t DMA_PeripheralDataSize; /*!< Specifies the Peripheral data width.
                                        This parameter can be a value of @ref DMA_peripheral_data_size */

  uint32_t DMA_MemoryDataSize;     /*!< Specifies the Memory data width.
                                        This parameter can be a value of @ref DMA_memory_data_size */

  uint32_t DMA_Mode;               /*!< Specifies the operation mode of the DMAy Streamx.
                                        This parameter can be a value of @ref DMA_circular_normal_mode
                                        @note The circular buffer mode cannot be used if the memory-to-memory
                                              data transfer is configured on the selected Stream */

  uint32_t DMA_Priority;           /*!< Specifies the software priority for the DMAy Streamx.
                                        This parameter can be a value of @ref DMA_priority_level */

  uint32_t DMA_FIFOMode;          /*!< Specifies if the FIFO mode or Direct mode will be used for the specified Stream.
                                        This parameter can be a value of @ref DMA_fifo_direct_mode
                                        @note The Direct mode (FIFO mode disabled) cannot be used if the 
                                               memory-to-memory data transfer is configured on the selected Stream */

  uint32_t DMA_FIFOThreshold;      /*!< Specifies the FIFO threshold level.
                                        This parameter can be a value of @ref DMA_fifo_threshold_level */

  uint32_t DMA_MemoryBurst;        /*!< Specifies the Burst transfer configuration for the memory transfers. 
                                        It specifies the amount of data to be transferred in a single non interruptable 
                                        transaction. This parameter can be a value of @ref DMA_memory_burst 
                                        @note The burst mode is possible only if the address Increment mode is enabled. */

  uint32_t DMA_PeripheralBurst;    /*!< Specifies the Burst transfer configuration for the peripheral transfers. 
                                        It specifies the amount of data to be transferred in a single non interruptable 
                                        transaction. This parameter can be a value of @ref DMA_peripheral_burst
                                        @note The burst mode is possible only if the address Increment mode is enabled. */  
}DMA_InitTypeDef;


//** DMA_channel 

#define DMA_Channel_0                     ((uint32_t)0x00000000)
#define DMA_Channel_1                     ((uint32_t)0x02000000)
#define DMA_Channel_3                     ((uint32_t)0x06000000)
#define DMA_Channel_4                     ((uint32_t)0x08000000)


//** DMA_data_transfer_direction 

#define DMA_DIR_PeripheralToMemory        ((uint32_t)0x00000000)
#define DMA_DIR_MemoryToPeripheral        ((uint32_t)0x00000040) 


#define DMA_PeripheralInc_Disable         ((uint32_t)0x00000000)


//** DMA_memory_incremented_mode 
#define DMA_MemoryInc_Enable              ((uint32_t)0x00000400)


/** DMA_peripheral_data_size */ 
#define DMA_PeripheralDataSize_Byte       ((uint32_t)0x00000000) 


// /** DMA_memory_data_size 
#define DMA_MemoryDataSize_Byte           ((uint32_t)0x00000000) 

///** DMA_circular_normal_mode 
#define DMA_Mode_Normal                   ((uint32_t)0x00000000) 

#define DMA_Priority_High                 ((uint32_t)0x00020000)

//** DMA_fifo_direct_mode 
#define DMA_FIFOMode_Disable              ((uint32_t)0x00000000) 

// /** DMA_fifo_threshold_level 
#define DMA_FIFOThreshold_1QuarterFull    ((uint32_t)0x00000000)

//@defgroup DMA_memory_burst*/
#define DMA_MemoryBurst_Single            ((uint32_t)0x00000000)

// /** DMA_peripheral_burst 
#define DMA_PeripheralBurst_Single        ((uint32_t)0x00000000)


#define DMA_FLAG_TEIF0                    ((uint32_t)0x10000008)
#define DMA_FLAG_TCIF0                    ((uint32_t)0x10000020)
#define DMA_FLAG_TEIF2                    ((uint32_t)0x10080000)
#define DMA_FLAG_TCIF2                    ((uint32_t)0x10200000)

// /** DMA_interrupt_enable_definitions 
#define DMA_IT_TC                         ((uint32_t)0x00000010)
#define DMA_IT_TE                         ((uint32_t)0x00000004)
#define DMA_IT_FE                         ((uint32_t)0x00000080)


// /** DMA_interrupts_definitions 
#define DMA_IT_TCIF6                      ((uint32_t)0x20208000)

/*  Function used to set the DMA configuration to the default reset state *****/ 
void DMA_DeInit(DMA_Stream_TypeDef* DMAy_Streamx);

/* Initialization and Configuration functions *********************************/
void DMA_Init(DMA_Stream_TypeDef* DMAy_Streamx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);

/* Data Counter functions *****************************************************/
void DMA_SetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx, uint16_t Counter);
uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx);

/* Interrupts and flags management functions **********************************/
FunctionalState DMA_GetCmdStatus(DMA_Stream_TypeDef* DMAy_Streamx);
FlagStatus DMA_GetFlagStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ITConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT, FunctionalState NewState);
void DMA_ClearITPendingBit(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);

#ifdef __cplusplus
}
#endif

#endif /*__STM32F4xx_DMA_H */
