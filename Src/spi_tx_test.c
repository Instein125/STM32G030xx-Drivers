/*
 * spi_tx_test.c
 *
 *  Created on: Jan 2, 2025
 *      Author: shres
 */

#include "stm32g030xx.h"
#include <string.h>

/*
 * pb11 -> SPI2_MOSI
 * pb12 -> SPI2_NSS   (Chip select)
 * pb13 -> SPI2_SCK
 * pb14 ->  SPI2_MISO
 * ALT function mode 0
 */
void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_Mode = GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_AltFunction = GPIO_AF_0;
	SPIPins.GPIO_PinConfig.GPIO_OpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_Speed = GPIO_SPEED_VHIGH;

	// Sclk
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(& SPIPins);
}

void SPI2_Inits(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_Mode = SPI_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPI_Config.SPI_DSF = SPI_DSF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}


int main()
{
	char user_data[] = "Hello world";

	// initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// initialize the SPI2 peripheral params
	SPI2_Inits();

	// makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralCtrl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//Disable the SPI2 peripheral
	SPI_PeripheralCtrl(SPI2, DISABLE);
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
