#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32g030xx.h"

/*
 * Configuration structure for a GPIO pin
 * Defines the configuration parameters for a GPIO pin.
 */
typedef struct {
    uint8_t GPIO_PinNumber;        /* GPIO pin number (e.g., 0 to 15 for STM32) see @GPIO_PinNum*/
    uint8_t GPIO_Mode;             /* GPIO mode (e.g., Input, Output, Alternate Function, Analog) see @GPIO_Mode*/
    uint8_t GPIO_Speed;            /* GPIO output speed (e.g., Low, Medium, High) see @GPIO_Output_Speed*/
    uint8_t GPIO_PuPdControl;      /* GPIO pull-up/pull-down configuration (e.g., No Pull, Pull-up, Pull-down) see @GPIO_Pull-Up/Pull-Down_Cfg*/
    uint8_t GPIO_OpType;           /* GPIO output type (e.g., Push-Pull, Open-Drain) see @GPIO_Output_Type*/
    uint8_t GPIO_AltFunction;      /* Alternate function mode (e.g., AF0 to AF15) see @GPIO_Alternate_Function*/
} GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 * Combines the base address of the GPIO port and the pin configuration.
 */
typedef struct {
    GPIO_RegDef_t *pGPIOx;         		/* Base address of the GPIO port (e.g., GPIOA, GPIOB) */
    GPIO_PinConfig_t GPIO_PinConfig; 	/* GPIO pin configuration settings */
} GPIO_Handle_t;

/*
 * @GPIO_PinNum Macros
 */
#define GPIO_PIN_NO_0        0
#define GPIO_PIN_NO_1        1
#define GPIO_PIN_NO_2        2
#define GPIO_PIN_NO_3        3
#define GPIO_PIN_NO_4        4
#define GPIO_PIN_NO_5        5
#define GPIO_PIN_NO_6        6
#define GPIO_PIN_NO_7        7
#define GPIO_PIN_NO_8        8
#define GPIO_PIN_NO_9        9
#define GPIO_PIN_NO_10       10
#define GPIO_PIN_NO_11       11
#define GPIO_PIN_NO_12       12
#define GPIO_PIN_NO_13       13
#define GPIO_PIN_NO_14       14
#define GPIO_PIN_NO_15       15

/*
 * @GPIO_Mode Macros
 * Define the various modes a GPIO pin can operate in.
 * These macros represent user-friendly options for setting GPIO_PinConfig_t.GPIO_Mode.
 */
#define GPIO_MODE_INPUT         0  /* Input Mode */
#define GPIO_MODE_OUTPUT        1  /* Output Mode */
#define GPIO_MODE_AF            2  /* Alternate Function Mode */
#define GPIO_MODE_ANALOG        3  /* Analog Mode */
#define GPIO_MODE_INT_FT        4  /* Input Interrupt Falling Edge trigger Mode */
#define GPIO_MODE_INT_RT        5  /* Input Interrupt Rising Edge trigger Mode */
#define GPIO_MODE_INT_RFT       6  /* Input Interrupt Rising Falling Edge trigger Mode */

/*
 * @GPIO_Output_Speed Macros
 * Represent the possible speeds of GPIO output signals.
 * Used for GPIO_PinConfig_t.GPIO_Speed.
 */
#define GPIO_SPEED_LOW          0  /* Low Speed */
#define GPIO_SPEED_MEDIUM       1  /* Medium Speed */
#define GPIO_SPEED_HIGH         2  /* High Speed */
#define GPIO_SPEED_VHIGH        3  /* Very High Speed */


/*
 * @GPIO_Pull-Up/Pull-Down_Cfg Macros
 * Define the pull resistor configurations for GPIO pins.
 * Used for GPIO_PinConfig_t.GPIO_PuPdControl.
 */
#define GPIO_NO_PUPD            0  /* No Pull-up or Pull-down */
#define GPIO_PULL_UP            1  /* Enable Pull-up Resistor */
#define GPIO_PULL_DOWN          2  /* Enable Pull-down Resistor */

/*
 * @GPIO_Output_Type Macros
 * Specify the electrical characteristics of the GPIO output.
 * Used for GPIO_PinConfig_t.GPIO_OpType.
 */
#define GPIO_OP_TYPE_PP         0  /* Push-Pull */
#define GPIO_OP_TYPE_OD         1  /* Open-Drain */

/*
 * @GPIO_Alternate_Function Macros
 * Define the alternate function modes for GPIO pins.
 * These are used when GPIO_PinConfig_t.GPIO_Mode is set to GPIO_MODE_AF.
 */
#define GPIO_AF_0               0  /* Alternate Function 0 */
#define GPIO_AF_1               1  /* Alternate Function 1 */
#define GPIO_AF_2               2  /* Alternate Function 2 */
#define GPIO_AF_3               3  /* Alternate Function 3 */
#define GPIO_AF_4               4  /* Alternate Function 4 */
#define GPIO_AF_5               5  /* Alternate Function 5 */
#define GPIO_AF_6               6  /* Alternate Function 6 */
#define GPIO_AF_7               7  /* Alternate Function 7 */
#define GPIO_AF_8               8  /* Alternate Function 8 */
#define GPIO_AF_9               9  /* Alternate Function 9 */
#define GPIO_AF_10              10  /* Alternate Function 10 */
#define GPIO_AF_11              11  /* Alternate Function 11 */
#define GPIO_AF_12              12  /* Alternate Function 12 */
#define GPIO_AF_13              13  /* Alternate Function 13 */
#define GPIO_AF_14              14  /* Alternate Function 14 */
#define GPIO_AF_15              15  /* Alternate Function 15 */



/*
 * APIs supported by the GPIO driver
 */

/**
 * @brief  Initializes the GPIO pin with the specified configuration.
 * @param  pGPIOHandle: Pointer to the GPIO handle structure containing the configuration.
 * @retval None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);													/*Initialize a GPIO pin*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);													/*Deinitialize a GPIO pin*/
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t en);									/*Enable or Disable the GPIO peripheral clock*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum);						/*Read from a GPIO pin*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);										/*Read from a GPIO port*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum, uint8_t value);			/*Write to a GPIO pin*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);							/*Write to a GPIO port*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum);							/*Toggle a GPIO Input pin*/
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t en);						/*GPIO Interrupt configuration*/
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNum);					/*GPIO Interrupt handling*/




#endif /* INC_GPIO_H_ */
