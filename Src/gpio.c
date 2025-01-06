#include "gpio.h"
/**
 * @brief  Initializes the GPIO pin with the specified configuration.
 * @param  pGPIOHandle: Pointer to the GPIO handle structure containing the configuration.
 * @retval None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0; // Temporary register variable

	// enable the peripheral clock
	GPIO_PeriClkCtrl(pGPIOHandle->pGPIOx, ENABLE);

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
		// Configure the MODER register to input mode for the corresponding pin
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_INT_FT){
			//1. Config FTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit as We want the falling edge so to make sure
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_INT_RT){
			//1. Config RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_INT_RFT){
			//1. Config FTSR and RTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

	    // 2. Configure the GPIO port selection in EXTI register
	    uint8_t exti_idx = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; // EXTICR index (each EXTICR covers 4 pins)
	    uint8_t exti_shift = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4) * 8; // Shift for the specific pin
	    uint8_t gpio_port_code = GPIO_BASE_TO_PORT_CODE(pGPIOHandle->pGPIOx);

	    // Update the corresponding EXTICR register (EXTICR1 - EXTICR4)
		EXTI->EXTICR[exti_idx] &= ~(0xFF << exti_shift); // Clear existing bits
		EXTI->EXTICR[exti_idx] |= (gpio_port_code << exti_shift); // Set the new port code


		//3. Enable the EXTI interrupt delivery using the IMR(Interrupt Mask Register)
		EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
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
 * @param  en: Enable/Disable flag (1 = ENABLE, 0 = DISABLE)
 * @retval None
 */
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t en){
	if(en == ENABLE){
		// Set 1 in NVIC_ISER
		*NVIC_ISER |= (1 << IRQNum);
	} else {
		*NVIC_ICER |= (1 << IRQNum);
	}
}

/*
 * @brief  Configure the GPIO interrupt priority
 * @param  IRQNum: Interrupt request number
 * @param  IRQPriority: Priority of the interrupt (0 = highest)
 * @retval None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority){
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
 * @brief  Handle the GPIO interrupt
 * @param  pinNum: GPIO pin number that triggered the interrupt
 * @retval None
 */
void GPIO_IRQHandling(uint8_t pinNum){
	// Check if the interrupt was triggered on the falling edge
    if(EXTI->FPR1 & (1 << pinNum)) {
    	EXTI->FPR1 = (1 << pinNum);  // Clear the falling edge pending flag for the specific pin
    }

    // Check if the interrupt was triggered on the rising edge
    if(EXTI->RPR1 & (1 << pinNum)) {
    	EXTI->RPR1 = (1 << pinNum);  // Clear the rising edge pending flag for the specific pin
    }
}
