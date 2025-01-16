/*
 * spi.h
 *
 *  Created on: Jan 1, 2025
 *      Author: shres
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32g030xx.h"

/*
 * Configuration structure for a SPI
 * Defines the configuration parameters for SPI.
 */
typedef struct {
   uint32_t SPI_Mode;              	/* Master/Slave selection */
   uint32_t SPI_BusConfig;         	/* Full-duplex/Half-duplex/Simplex */
   uint32_t SPI_SclkSpeed;         	/* Clock speed */
   uint32_t SPI_DSF;      			/* Data Size Format*/
   uint32_t SPI_CPHA;         		/* Clock phase - data capture edge */
   uint32_t SPI_CPOL;              	/* Clock polarity IDLE state */
   uint32_t SPI_SSM;         		/* Slave select  Management*/
} SPI_Config_t;

/*
 * Handle structure for a SPI
 * Combines the base address of the SPIx and the SPI configuration.
 */
typedef struct {
    SPI_RegDef_t *pSPIx;         		/* Base address of the GPIO port (e.g., GPIOA, GPIOB) */
    SPI_Config_t SPI_Config; 	/* SPI configuration settings */
} SPI_Handle_t;


/*
 * @GPIO_Mode Macros
 * Define the various modes a SPI can operate in.
 */
#define SPI_MODE_SLAVE        0  /* Slave Mode */
#define SPI_MODE_MASTER       1  /* Master Mode */

/*
 * @SPI_BusConfig Macros
 * Represent the possible bus configuration of SPI.
 */
#define SPI_BUS_CONFIG_FD          		0  /* Full-duplex */
#define SPI_BUS_CONFIG_HD       		1  /* Half-duplex */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   2  /* Simplex receive only */

/*
 * @SPI_SclkSpeed Macros
 * Represent the possible Baud rate of SPI.
 */
#define SPI_SCLK_SPEED_DIV2          		0  /* Peripheral clock divided by 2 */
#define SPI_SCLK_SPEED_DIV4          		1  /* Peripheral clock divided by 4 */
#define SPI_SCLK_SPEED_DIV8          		2  /* Peripheral clock divided by 8 */
#define SPI_SCLK_SPEED_DIV16          		3  /* Peripheral clock divided by 16 */
#define SPI_SCLK_SPEED_DIV32          		4  /* Peripheral clock divided by 32 */
#define SPI_SCLK_SPEED_DIV64          		5  /* Peripheral clock divided by 64 */
#define SPI_SCLK_SPEED_DIV128          		6  /* Peripheral clock divided by 128 */
#define SPI_SCLK_SPEED_DIV256          		7  /* Peripheral clock divided by 254 */

/*
 * @SPI_DFF Macros
 * Represent the data format supported for communication
 */
#define SPI_DSF_8BITS						7 /* 8 bit data size */
#define SPI_DSF_16Bits						15 /* 16 bit data size */

/*
 * @SPI_CPHA Macros
 * Represent the Clock phase - data capture edge
 */
#define SPI_CPHA_LOW						0 /* first clock transition is the first data capture edge */
#define SPI_CPHA_HIGH						1 /* second clock transition is the first data capture edge */

/*
 * @SPI_CPOL Macros
 * Represent the Clock polarity IDLE state
 */
#define SPI_CPOL_LOW						0 /* CK to 0 when idle */
#define SPI_CPOL_HIGH						1 /* CK to 1 when idle */

/*
 * @SPI_CPOL Macros
 * Represent the Clock polarity IDLE state
 */
#define SPI_CPOL_LOW						0 /* CK to 0 when idle */
#define SPI_CPOL_HIGH						1 /* CK to 1 when idle */

/*
 * @SPI_SSM Macros
 * Represent whether slave selection is managed by software or hardware
 */
#define SPI_SSM_DI					0 /* Software slave management disable*/
#define SPI_SSM_EN					1 /* Software slave management enable*/

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG       (1 << SPI_SR_TXE)   // Transmit buffer empty flag
#define SPI_RXNE_FLAG      (1 << SPI_SR_RXNE)  // Receive buffer not empty flag
#define SPI_BSY_FLAG       (1 << SPI_SR_BSY)   // Busy flag
#define SPI_OVR_FLAG       (1 << SPI_SR_OVR)   // Overrun flag
#define SPI_MODF_FLAG      (1 << SPI_SR_MODF)  // Mode fault flag
#define SPI_CRCERR_FLAG    (1 << SPI_SR_CRCERR) // CRC error flag
#define SPI_FRE_FLAG       (1 << SPI_SR_FRE)   // Frame format error flag
#define SPI_TIFRFE_FLAG    (1 << SPI_SR_TIFRFE) // TI frame format error flag

/*
 * APIs supported by the SPI driver
 */

/**
 * @brief  Initializes the GPIO pin with the specified configuration.
 * @param  pGPIOHandle: Pointer to the GPIO handle structure containing the configuration.
 * @retval None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);													/*Initialize a GPIO pin*/
void SPI_DeInit(SPI_RegDef_t *pSPIx);													/*Deinitialize a GPIO pin*/

void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t en);									/*Enable or Disable the GPIO peripheral clock*/
void SPI_PeripheralCtrl(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t en);
/*
 * Data send and receive
 * Two mode Blocking(non interrupt mode) and non blocking(interrupt mode)
 * DMA mode also
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handeling
 */
void SPI_IRQConfig(uint8_t IRQNum, uint8_t en);						/*GPIO Interrupt configuration*/
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);					/*GPIO Interrupt handling*/

#endif /* SPI_H_ */
