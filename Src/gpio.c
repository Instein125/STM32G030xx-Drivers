#include "gpio.h"
/**
 * @brief  Initializes the GPIO pin with the specified configuration.
 * @param  pGPIOHandle: Pointer to the GPIO handle structure containing the configuration.
 * @retval None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0; // Temporary register variable

	// Configure GPIO Mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_Mode <= GPIO_MODE_ANALOG) {
		// Non-interrupt mode (Input, Output, Alternate Function, Analog)
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Mode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear MODER bits
		pGPIOHandle->pGPIOx->MODER |= temp; // Set new mode
	} else {
		// Interrupt mode (to be implemented)
	}
	temp = 0;

	// Configure GPIO Output Type
	if (pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_OUTPUT) {
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_OpType
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear OTYPER bit
		pGPIOHandle->pGPIOx->OTYPER |= temp; // Set output type
	}
	temp = 0;

	// Configure GPIO Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Speed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear OSPEEDR bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // Set speed
	temp = 0;

	// Configure GPIO Pull-up/Pull-down
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear PUPDR bits
	pGPIOHandle->pGPIOx->PUPDR |= temp; // Set pull-up/pull-down configuration
	temp = 0;

	// Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_AF){
		uint8_t regIdx, pinPos;
		regIdx = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		pinPos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[regIdx] |= (pGPIOHandle->GPIO_PinConfig.GPIO_AltFunction << (4 * pinPos));
	}
}

/**
 * @brief  Deinitializes the GPIO port by resetting all configurations.
 * @param  pGPIOx: Base address of the GPIO peripheral to be reset.
 * @retval None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA)	GPIOA_REG_RST();
	else if(pGPIOx == GPIOB) GPIOB_REG_RST();
	else if(pGPIOx == GPIOC) GPIOC_REG_RST();
	else if(pGPIOx == GPIOD) GPIOD_REG_RST();
	else if(pGPIOx == GPIOF) GPIOF_REG_RST();
}

/**
 * @brief  Enables or disables the clock for the specified GPIO peripheral.
 * @param  pGPIOx: Base address of the GPIO peripheral.
 * @param  en: Enable/disable flag (1 to enable, 0 to disable).
 * @retval None
 */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t en){
	if(en == ENABLE){
		if(pGPIOx == GPIOA)	GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOF) GPIOF_PCLK_EN();
	}else{
		if(pGPIOx == GPIOA)	GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC) GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD) GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOF) GPIOF_PCLK_DI();
	}
}

/**
 * @brief  Reads the logic level from a specific GPIO pin.
 * @param  pGPIOx: Base address of the GPIO peripheral.
 * @param  pinNum: Pin number to read (0–15).
 * @retval Logic level of the pin (0 or 1).
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum){
	uint8_t value = (uint8_t)((pGPIOx->IDR >> pinNum) & 0x01 );
	return value;
}

/**
 * @brief  Reads the input data from the entire GPIO port.
 * @param  pGPIOx: Base address of the GPIO peripheral.
 * @retval 16-bit data representing the logic level of all pins.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/**
 * @brief  Writes a logic level to a specific GPIO pin.
 * @param  pGPIOx: Base address of the GPIO peripheral.
 * @param  pinNum: Pin number to write to (0–15).
 * @param  value: Logic level to write (0 or 1).
 * @retval None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum, uint8_t value){
	if (value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << (pinNum));
	} else if (value == GPIO_PIN_RST){
		pGPIOx->ODR &= ~(1 << pinNum);
	}
}

/**
 * @brief  Writes an 8-bit value to the entire GPIO port.
 * @param  pGPIOx: Base address of the GPIO peripheral.
 * @param  value: 8-bit value to write to the port.
 * @retval None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

/**
 * @brief  Toggles the logic level of a specific GPIO pin.
 * @param  pGPIOx: Base address of the GPIO peripheral.
 * @param  pinNum: GPIO pin number (0-15)
 * @retval None
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum){
	pGPIOx->ODR ^= (1 << pinNum);
}

/**
 * @brief  Configure the GPIO interrupt
 * @param  IRQNum: Interrupt request number
 * @param  IRQPriority: Priority of the interrupt (0 = highest)
 * @param  en: Enable/Disable flag (1 = Enable, 0 = Disable)
 * @retval None
 */
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t IRQPriority, uint8_t en){

}

/**
 * @brief  Handle the GPIO interrupt
 * @param  pinNum: GPIO pin number that triggered the interrupt
 * @retval None
 */
void GPIO_IRQHandling(uint8_t pinNum){

}
