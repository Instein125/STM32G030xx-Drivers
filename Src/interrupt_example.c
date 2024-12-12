#include <stdint.h>
#include <string.h>
#include "stm32g030xx.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void EXTI4_15_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_12);
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_3);
}


int main(void){

	GPIO_Handle_t GpioLed;
	memset(&GpioLed, 0, sizeof(GpioLed));

	SYSCFG_PCLK_EN();

 	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GpioLed.GPIO_PinConfig.GPIO_Mode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_Speed = GPIO_SPEED_MEDIUM;
	GpioLed.GPIO_PinConfig.GPIO_OpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClkCtrl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);


	GPIO_Handle_t GpioButton;
	memset(&GpioButton, 0, sizeof(GpioButton));

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioButton.GPIO_PinConfig.GPIO_Mode = GPIO_MODE_INT_RT;
	GpioButton.GPIO_PinConfig.GPIO_Speed = GPIO_SPEED_VHIGH;
	GpioButton.GPIO_PinConfig.GPIO_OpType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PULL_DOWN;

	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioButton);

	GPIO_IRQPriorityConfig(EXTI4_15_IRQn, NVIC_IRQ_PRIORITY_3);
	GPIO_IRQConfig(EXTI4_15_IRQn, ENABLE);

	while(1);

}


