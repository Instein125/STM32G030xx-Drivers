/*
 * spi.c
 *
 *  Created on: Jan 1, 2025
 *      Author: shres
 */

#include "spi.h"

/**
 * @brief  Initializes the SPI with the specified configuration.
 * @param  pSPIHandle: Pointer to the SPI handle structure containing the configuration.
 * @retval None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

}

/**
 * @brief  Deinitializes the SPI by resetting all configurations.
 * @param  pSPIx: Base address of the SPI peripheral to be reset.
 * @retval None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

}

/**
 * @brief  Enables or disables the clock for the specified SPI peripheral.
 * @param  pSPIx: Base address of the SPI peripheral.
 * @param  en: Enable/disable flag (1 to enable, 0 to disable).
 * @retval None
 */
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t en){
	if(en == ENABLE){
		if(pSPIx == SPI1)	SPI1_PCLK_EN();
		else if(pSPIx == SPI2) SPI2_PCLK_EN();
	}else{
		if(pSPIx == SPI1)	SPI1_PCLK_DI();
		else if(pSPIx == SPI2) SPI2_PCLK_DI();
	}
}

/**
 * @brief  Send the data
 * @param  pSPIx: Base address of the SPI peripheral.
 * @param  pTxBuffer: Buffer containing the data to be transmitted
 * @param  Len: Size of the data to be transmitted
 * @retval None
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

}

/**
 * @brief  Receive the data
 * @param  pSPIx: Base address of the SPI peripheral.
 * @param  pRxBuffer: Buffer to store the data
 * @param  Len: Size of the data to be received
 * @retval None
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

}

/**
 * @brief  Configure the SPI interrupt
 * @param  IRQNum: Interrupt request number
 * @param  en: Enable/Disable flag (1 = ENABLE, 0 = DISABLE)
 * @retval None
 */
void SPI_IRQConfig(uint8_t IRQNum, uint8_t en){
	if(en == ENABLE){
		// Set 1 in NVIC_ISER
		*NVIC_ISER |= (1 << IRQNum);
	} else {
		*NVIC_ICER |= (1 << IRQNum);
	}
}

/*
 * @brief  Configure the SPI interrupt priority
 * @param  IRQNum: Interrupt request number
 * @param  IRQPriority: Priority of the interrupt (0 = highest)
 * @retval None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority){
	// Make sure that the IRQ priority is within the valid range (0-3 for Cortex-M0+)
	if (IRQPriority < 4) {
		// Each register covers 4 interrupt priorities (IRQNum 0-3, 4-7, etc.)
		uint8_t registerIndex = IRQNum / 4;
		uint8_t iprx_section = (IRQNum % 4);

		uint8_t shift_amt = (8*iprx_section) + (8-2);

		*(NVIC_IPR + (registerIndex)) |= IRQPriority << shift_amt;
	}
}

/**
 * @brief  Handle the SPI interrupt
 * @param  pSPIHandle: Pointer to the SPI handle structure
 * @retval None
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

}

