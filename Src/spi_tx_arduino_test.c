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

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}

void SPI1_Inits(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI1;
	SPI2handle.SPI_Config.SPI_Mode = SPI_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPI_Config.SPI_DSF = SPI_DSF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}

void GPIO_buttonInit(void){
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioButton.GPIO_PinConfig.GPIO_Mode = GPIO_MODE_INPUT;
	GpioButton.GPIO_PinConfig.GPIO_Speed = GPIO_SPEED_MEDIUM;
	GpioButton.GPIO_PinConfig.GPIO_OpType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioButton);
}

void delay(void){
	for(uint32_t i = 0; i < 500000; i++);
}


int main()
{
	char user_data[] = "Hello world";

	GPIO_buttonInit();

	// initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	// initialize the SPI1 peripheral params
	SPI1_Inits();

	/*
	 * Making the SSOE 1 does NSS output enable
	 * the NSS pin is automatically managed by the hardware
	 * i.e when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI1, ENABLE);


	while (1) {
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_12));
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralCtrl(SPI1, ENABLE);

		// sending the data length information for slave
		uint8_t datalen = strlen(user_data);
		SPI_SendData(SPI1, &datalen, 1);

		SPI_SendData(SPI1, (uint8_t*) user_data, strlen(user_data));

		// wait until the SPI is busy
		while( SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));
		//Disable the SPI1 peripheral
		SPI_PeripheralCtrl(SPI1, DISABLE);
	}
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
