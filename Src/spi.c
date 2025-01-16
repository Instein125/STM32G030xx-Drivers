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
	// congifure the SPI_CR1 register
	uint32_t tempreg = 0;

	//enable the peripheral clock
	SPI_PeriClkCtrl(pSPIHandle->pSPIx, ENABLE);

	// 1. configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_Mode << 2;

	// 2. Configure the bus config
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// Full-Duplex mode (default), BIDIMODE = 0
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// Half-Duplex mode, BIDIMODE = 1
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// Simplex mode (Receive only), BIDIMODE = 0, RXONLY = 1
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI clock speed (Baud Rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR_Pos;

	// 4. Configure the SPI Clock Phase (CPHA)
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// 5. Configure the SPI Clock Polarity (CPOL)
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the SPI Software Slave Management (SSM)
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	// Write to SPI_CR1 register
	pSPIHandle->pSPIx->CR1 = tempreg;

	// configure the SPI_CR2 register
	tempreg = 0;

	// 1. Configure the Data Frame Size (DS[3:0] in CR2)
	tempreg |= (pSPIHandle->SPI_Config.SPI_DSF << SPI_CR2_DS_Pos);

	// Write to SPI_CR2 register
	pSPIHandle->pSPIx->CR2 = tempreg;
}

/**
 * @brief  Deinitializes the SPI by resetting all configurations.
 * @param  pSPIx: Base address of the SPI peripheral to be reset.
 * @retval None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if (pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2) SPI2_REG_RESET();
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

void SPI_PeripheralCtrl(SPI_RegDef_t *pSPIx, uint8_t en){
	if(en == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t en){
	if(en == ENABLE){
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		} else {
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t en){
	if(en == ENABLE){
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		} else {
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}

/**
 * @brief  Send the data
 * @param  pSPIx: Base address of the SPI peripheral.
 * @param  pTxBuffer: Buffer containing the data to be transmitted
 * @param  Len: Size of the data to be transmitted
 * @retval None
 *
 * @note  This is blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while (Len > 0){
		// 1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check the DFS bit in CR2
		if (((pSPIx->CR2 & (0xF << SPI_CR2_DS_Pos)) >> SPI_CR2_DS_Pos) == SPI_DSF_16Bits){
			// 16 bit DFS
			// 1. Load 16 bits of data into the DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2; // Reduce length by 2 bytes
			(uint16_t*)pTxBuffer++; // increment the buffer by 2 bytes(done by uibt16_t typecast)
		} else if ((((pSPIx->CR2 & (0xF << SPI_CR2_DS_Pos)) >> SPI_CR2_DS_Pos) == SPI_DSF_8BITS)) {
			// 8 bit DFS
			// 1. Load 8 bits of data into the DR register
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
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

