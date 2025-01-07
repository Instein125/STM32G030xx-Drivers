/*
 * spi_tx_test.c
 *
 *  Created on: Jan 2, 2025
 *      Author: shres
 */

#include "stm32g030xx.h"
#include <string.h>

/*
 * PA2 -> SPI1_MOSI
 * PA4 -> SPI1_NSS   (Chip select)
 * PA1 -> SPI1_SCK
 * PA6 ->  SPI1_MISO
 * ALT function mode 0
 */
void SPI1_GPIOInits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_Mode = GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_AltFunction = GPIO_AF_0;
	SPIPins.GPIO_PinConfig.GPIO_OpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_Speed = GPIO_SPEED_VHIGH;

	// Sclk
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&SPIPins);

//	// MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
//	GPIO_Init(&SPIPins);
//
//	// NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
//	GPIO_Init(& SPIPins);
}

void SPI1_Inits(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI1;
	SPI2handle.SPI_Config.SPI_Mode = SPI_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPI2handle.SPI_Config.SPI_DSF = SPI_DSF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}


int main()
{
	char user_data[] = "Hello world";

	// initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	// initialize the SPI1 peripheral params
	SPI1_Inits();

	// makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI1, ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralCtrl(SPI1, ENABLE);

	SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

	//Disable the SPI1 peripheral
	SPI_PeripheralCtrl(SPI1, DISABLE);
	while(1);
	return 0;
}


/*
 * Why Configure SSI When SSM is Enabled?
When SSM is enabled, the SPI peripheral does not monitor the physical NSS pin. However, internally, the NSS signal still plays a role in the SPI's operation. If the NSS signal (emulated by the SSI bit) is not configured correctly, it can lead to a Mode Fault (MODF) error.

MODF Error:
A Mode Fault (MODF) occurs when:

- The SPI is in master mode.
- The SPI detects that the NSS signal goes low unexpectedly (indicating another master is trying to control the SPI bus).
Even with SSM enabled, the SPI still requires the NSS signal to be internally high (inactive) to operate as a master.

How the SSI Bit Helps:
- By setting SSI = 1, you emulate the NSS signal being high (inactive).
- This prevents the SPI peripheral from detecting a false MODF error.
 *
 */
