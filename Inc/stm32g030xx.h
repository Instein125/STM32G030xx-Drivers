#ifndef INC_STM32G030XX_H_
#define INC_STM32G030XX_H_

#include <stdint.h>

#define __vo volatile


/*
 * ARM Cortex M0+ Processor NVIC Register Addresses
 */
#define NVIC_ISER_BASE_ADDR   		(0xE000E100UL) /* Base address for the Interrupt Set-Enable Registers */
#define NVIC_ICER_BASE_ADDR   		(0xE000E180UL) /* Base address for the Interrupt Clear-Enable Registers */
#define NVIC_IPR_BASE_ADDR    		(0xE000E400UL) /* Base address for the Interrupt Priority Registers */

#define NVIC_ISER             ((__vo uint32_t *)NVIC_ISER_BASE_ADDR)
#define NVIC_ICER             ((__vo uint32_t *)NVIC_ICER_BASE_ADDR)
#define NVIC_IPR              ((__vo uint8_t *)NVIC_IPR_BASE_ADDR)

/*
 * Base addresses of Flash, ROM and SRAM memories
 */

#define FLASH_BASEADDR				0x08000000U 	/*Base address for the Flash(Main Memory) Memory */
#define SRAM_BASEADDR				0x20000000U		/*Base address for the SRAM Memory */
#define ROM_BASEADDR				0x1FFFF000U		/*Base address for the ROM(System Memory) Memory */

/*
 * AHBx, APBx and IOPORT bus peripheral base addresses
 */

#define PERIPH_BASEADDR				0x40000000U		/*Base address of peripheral bus*/
#define APBPERIPH_BASEADDR			PERIPH_BASEADDR	/*Base address of APB peripheral bus*/
#define AHBPERIPH_BASEADDR			0x40020000U		/*Base address of AHB peripheral bus*/
#define IOPORT_BASEADDR				0x50000000U		/*Base address of IOPORT peripheral bus*/

/*
 * Base address of peripherals which are hanging on the APB1 bus for STM32G030
 */
#define TIM3_BASEADDR               (APBPERIPH_BASEADDR + 0x0400) /* Base address of TIM3 */
#define TIM14_BASEADDR              (APBPERIPH_BASEADDR + 0x2000) /* Base address of TIM14 */
#define RTC_BASEADDR                (APBPERIPH_BASEADDR + 0x2800) /* Base address of RTC and TAMP */
#define WWDG_BASEADDR               (APBPERIPH_BASEADDR + 0x2C00) /* Base address of Window Watchdog (WWDG) */
#define IWDG_BASEADDR               (APBPERIPH_BASEADDR + 0x3000) /* Base address of Independent Watchdog (IWDG) */
#define SPI2_BASEADDR               (APBPERIPH_BASEADDR + 0x3800) /* Base address of SPI2 */
#define USART2_BASEADDR             (APBPERIPH_BASEADDR + 0x4400) /* Base address of USART2 */
#define I2C1_BASEADDR               (APBPERIPH_BASEADDR + 0x5400) /* Base address of I2C1 */
#define I2C2_BASEADDR               (APBPERIPH_BASEADDR + 0x5800) /* Base address of I2C2 */
#define USB_BASEADDR               	(APBPERIPH_BASEADDR + 0x5C00) /* Base address of USB */
#define PWR_BASEADDR                (APBPERIPH_BASEADDR + 0x7000) /* Base address of PWR */
#define TAMP_BASEADDR               (APBPERIPH_BASEADDR + 0xB000) /* Base address of TAMP + Backup Registers */
#define SYSCFG_BASEADDR             (APBPERIPH_BASEADDR + 0x10000) /* Base address of SYSCFG */
#define ADC_BASEADDR                (APBPERIPH_BASEADDR + 0x12400) /* Base address of ADC */
#define TIM1_BASEADDR               (APBPERIPH_BASEADDR + 0x12C00) /* Base address of TIM1 */
#define SPI1_BASEADDR               (APBPERIPH_BASEADDR + 0x13000) /* Base address of SPI1/I2S1 */
#define USART1_BASEADDR             (APBPERIPH_BASEADDR + 0x13800) /* Base address of USART1 */
#define TIM16_BASEADDR              (APBPERIPH_BASEADDR + 0x14400) /* Base address of TIM16 */
#define TIM17_BASEADDR              (APBPERIPH_BASEADDR + 0x14800) /* Base address of TIM17 */



/*
 * Base address of peripherals on the AHB bus
 */
#define DMA1_BASEADDR             	(AHBPERIPH_BASEADDR + 0x0000) /* Base address of DMA1 */
#define RCC_BASEADDR            	(AHBPERIPH_BASEADDR + 0x1000) /* Base address of RCC */
#define EXTI_BASEADDR            	(AHBPERIPH_BASEADDR + 0x1800) /* Base address of EXTI */
#define CRC_BASEADDR                (AHBPERIPH_BASEADDR + 0x3000) /* Base address of Clock Recovery System (CRC) */

/*
 * Base address of peripherals on the IOPORT bus
 */
#define GPIOA_BASEADDR             	(IOPORT_BASEADDR + 0x0000) /* Base address of DMA1 */
#define GPIOB_BASEADDR            	(IOPORT_BASEADDR + 0x0400) /* Base address of RCC */
#define GPIOC_BASEADDR            	(IOPORT_BASEADDR + 0x0800) /* Base address of EXTI */
#define GPIOD_BASEADDR              (IOPORT_BASEADDR + 0x0C00) /* Base address of Clock Recovery System (CRC) */
#define GPIOF_BASEADDR              (IOPORT_BASEADDR + 0x1400) /* Base address of Clock Recovery System (CRC) */


/********************************	Peripheral Register definition structures		*********************************/
/*
 * GPIO_RegDef_t structure defines the memory layout of the GPIO (General Purpose Input/Output) peripheral
 */
typedef struct{
	__vo uint32_t MODER;        /* GPIO port mode register              Address offset: 0x00 */
	__vo uint32_t OTYPER;       /* GPIO port output type register       Address offset: 0x04 */
	__vo uint32_t OSPEEDR;      /* GPIO port output speed register      Address offset: 0x08 */
	__vo uint32_t PUPDR;        /* GPIO port pull-up/pull-down register Address offset: 0x0C */
	__vo uint32_t IDR;          /* GPIO port input data register        Address offset: 0x10 */
	__vo uint32_t ODR;          /* GPIO port output data register       Address offset: 0x14 */
	__vo uint32_t BSRR;         /* GPIO port bit set/reset register     Address offset: 0x18 */
	__vo uint32_t LCKR;         /* GPIO port configuration lock register Address offset: 0x1C */
	__vo uint32_t AFR[2];       /* GPIO alternate function registers    AFR[0]: Low (0x20), AFR[1]: High (0x24) */
	__vo uint32_t BRR;          /* GPIO port bit reset register         Address offset: 0x28 */
} GPIO_RegDef_t;

/*
 * RCC_RegDef_t structure defines the memory layout of the RCC peripheral
 */
typedef struct {
    __vo uint32_t CR;            /* Clock control register                      Address offset: 0x00 */
    __vo uint32_t ICSCR;         /* Internal clock sources calibration register Address offset: 0x04 */
    __vo uint32_t CFGR;          /* Clock configuration register                Address offset: 0x08 */
    __vo uint32_t PLLCFGR;       /* PLL configuration register                  Address offset: 0x0C */
    __vo uint32_t RESERVED1;     /* Reserved                                    Address offset: 0x10 */
    __vo uint32_t RESERVED2;     /* Reserved                                    Address offset: 0x14 */
    __vo uint32_t CIER;          /* Clock interrupt enable register             Address offset: 0x18 */
    __vo uint32_t CIFR;          /* Clock interrupt flag register               Address offset: 0x1C */
    __vo uint32_t CICR;          /* Clock interrupt clear register              Address offset: 0x20 */
    __vo uint32_t IOPRSTR;       /* GPIO reset register                         Address offset: 0x24 */
    __vo uint32_t AHBRSTR;       /* AHB peripheral reset register               Address offset: 0x28 */
    __vo uint32_t APBRSTR1;      /* APB1 peripheral reset register              Address offset: 0x2C */
    __vo uint32_t APBRSTR2;      /* APB2 peripheral reset register              Address offset: 0x30 */
    __vo uint32_t IOPENR;        /* GPIO clock enable register                  Address offset: 0x34 */
    __vo uint32_t AHBENR;        /* AHB peripheral clock enable register        Address offset: 0x38 */
    __vo uint32_t APBENR1;       /* APB1 peripheral clock enable register       Address offset: 0x3C */
    __vo uint32_t APBENR2;       /* APB2 peripheral clock enable register       Address offset: 0x40 */
    __vo uint32_t IOPSMENR;      /* GPIO clock sleep mode enable register       Address offset: 0x44 */
    __vo uint32_t AHBSMENR;      /* AHB clock sleep mode enable register        Address offset: 0x48 */
    __vo uint32_t APBSMENR1;     /* APB1 clock sleep mode enable register       Address offset: 0x4C */
    __vo uint32_t APBSMENR2;     /* APB2 clock sleep mode enable register       Address offset: 0x50 */
    __vo uint32_t CCIPR;         /* Peripherals independent Clock configuration register                Address offset: 0x54 */
    __vo uint32_t RESERVED3;     /* Reserved                                    Address offset: 0x58 */
    __vo uint32_t BDCR;          /* Backup domain control register              Address offset: 0x5C */
    __vo uint32_t CSR;           /* Control/status register                     Address offset: 0x60 */
} RCC_RegDef_t;

/*
 * EXTI_RegDef_t structure defines the structure for EXTI
 */
typedef struct {
    __vo uint32_t RTSR1;       /* Rising Trigger selection register 1    Address offset: 0x00 */
    __vo uint32_t FTSR1;       /* Falling Trigger selection register 1   Address offset: 0x04 */
    __vo uint32_t SWIER1;      /* Software interrupt event register 1    Address offset: 0x08 */
    __vo uint32_t RPR1;        /* Rising edge pending register 1         Address offset: 0x0C */
    __vo uint32_t FPR1;        /* Falling edge pending register 1        Address offset: 0x10 */
    uint32_t RESERVED1[19];    /* Reserved                               Address offset: 0x14-0x5C */
    __vo uint32_t EXTICR[4];   /* External interrupt configuration registers Address offset: 0x60-0x6C */
    uint32_t RESERVED2[4];     /* Reserved                               Address offset: 0x70-0x7C */
    __vo uint32_t IMR1;        /* Interrupt mask register 1              Address offset: 0x80 */
    __vo uint32_t EMR1;        /* Event mask register 1                  Address offset: 0x84 */
} EXTI_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*) GPIOF_BASEADDR)

#define RCC				((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI 			((EXTI_RegDef_t*) EXTI_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		( RCC->IOPENR |= (1 << 0) )	// Enable clock for GPIOA
#define GPIOB_PCLK_EN()		( RCC->IOPENR |= (1 << 1) )	// Enable clock for GPIOB
#define GPIOC_PCLK_EN()		( RCC->IOPENR |= (1 << 2) )	// Enable clock for GPIOC
#define GPIOD_PCLK_EN()		( RCC->IOPENR |= (1 << 3) )	// Enable clock for GPIOD
#define GPIOF_PCLK_EN()		( RCC->IOPENR |= (1 << 5) )	// Enable clock for GPIOF

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		( RCC->IOPENR &= ~(1 << 0) )	// Disable clock for GPIOA
#define GPIOB_PCLK_DI()		( RCC->IOPENR &= ~(1 << 1) )	// Disable clock for GPIOB
#define GPIOC_PCLK_DI()		( RCC->IOPENR &= ~(1 << 2) )	// Disable clock for GPIOC
#define GPIOD_PCLK_DI()		( RCC->IOPENR &= ~(1 << 3) )	// Disable clock for GPIOD
#define GPIOF_PCLK_DI()		( RCC->IOPENR &= ~(1 << 5) )	// Disable clock for GPIOF

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APBENR2 |= (1 << 12) )	// Enable clock for SPI1
#define SPI2_PCLK_EN()		( RCC->APBENR1 |= (1 << 14) )	// Enable clock for SPI2

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		( RCC->APBENR2 &= ~(1 << 12) )	// Disable clock for SPI1
#define SPI2_PCLK_DI()		( RCC->APBENR1 &= ~(1 << 14) )	// Disable clock for SPI2

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APBENR1 |= (1 << 21) )	// Enable clock for I2C1
#define I2C2_PCLK_EN()		( RCC->APBENR1 |= (1 << 22) )	// Enable clock for I2C2

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		( RCC->APBENR1 &= ~(1 << 21) )	// Disable clock for I2C1
#define I2C2_PCLK_DI()		( RCC->APBENR1 &= ~(1 << 22) )	// Disable clock for I2C2

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		( RCC->APBENR2 |= (1 << 14) )	// Enable clock for USART1
#define USART2_PCLK_EN()		( RCC->APBENR1 |= (1 << 17) )	// Enable clock for USART2

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		( RCC->APBENR2 &= ~(1 << 14) )	// Disable clock for USART1
#define USART2_PCLK_DI()		( RCC->APBENR1 &= ~(1 << 17) )	// Disable clock for USART2

/*
 * Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()		( RCC->APBENR2 |= (1 << 0) )		// Enable clock for ADC1

/*
 * Clock disable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()		( RCC->APBENR2 &= ~(1 << 0) )		// Disable clock for ADC1
/*
 * Clock enable macros for ADCx peripherals
 */

#define ADC_PCLK_EN()		( RCC->APBENR2 |= (1 << 20) )		// Enable clock for ADC1

/*
 * Clock disable macros for ADCx peripherals
 */

#define ADC_PCLK_DI()		( RCC->APBENR2 &= ~(1 << 20) )		// Disable clock for ADC1

/*
 * Clock enable macros for TIMx peripherals
 */

#define TIM1_PCLK_EN()		( RCC->APBENR2 |= (1 << 11) )		// Enable clock for TIM1
#define TIM3_PCLK_EN()		( RCC->APBENR1 |= (1 << 1) )		// Enable clock for TIM3
#define TIM14_PCLK_EN()		( RCC->APBENR2 |= (1 << 15) )		// Enable clock for TIM14
#define TIM16_PCLK_EN()		( RCC->APBENR2 |= (1 << 17) )		// Enable clock for TIM16
#define TIM17_PCLK_EN()		( RCC->APBENR2 |= (1 << 18) )		// Enable clock for TIM17

/*
 * Clock disable macros for TIMx peripherals
 */

#define TIM1_PCLK_DI()		( RCC->APBENR2 &= ~(1 << 11) )		// Disable clock for TIM1
#define TIM3_PCLK_DI()		( RCC->APBENR1 &= ~(1 << 1) )		// Disable clock for TIM3
#define TIM14_PCLK_DI()		( RCC->APBENR2 &= ~(1 << 15) )		// Disable clock for TIM14
#define TIM16_PCLK_DI()		( RCC->APBENR2 &= ~(1 << 17) )		// Disable clock for TIM16
#define TIM17_PCLK_DI()		( RCC->APBENR2 &= ~(1 << 18) )		// Disable clock for TIM17

/*
 * Clock enable macros for DMAx peripherals
 */

#define DMA1_PCLK_EN()		( RCC->AHBENR |= (1 << 0) )		// Enable clock for DMA1

/*
 * Clock disable macros for DMAx peripherals
 */

#define DMA1_PCLK_DI()		( RCC->AHBENR &= ~(1 << 0) )		// Disable clock for DMA1

/*
 * Macros to reset GPIOx peripherals
 */
// GPIO Reset Macros
#define GPIOA_REG_RST()    do{ (RCC->IOPRSTR |= (1 << 0)); (RCC->IOPRSTR &= ~(1 << 2)); }while(0)
#define GPIOB_REG_RST()    do{ (RCC->IOPRSTR |= (1 << 1)); (RCC->IOPRSTR &= ~(1 << 3)); }while(0)
#define GPIOC_REG_RST()    do{ (RCC->IOPRSTR |= (1 << 2)); (RCC->IOPRSTR &= ~(1 << 4)); }while(0)
#define GPIOD_REG_RST()    do{ (RCC->IOPRSTR |= (1 << 3)); (RCC->IOPRSTR &= ~(1 << 5)); }while(0)
#define GPIOF_REG_RST()    do{ (RCC->IOPRSTR |= (1 << 5)); (RCC->IOPRSTR &= ~(1 << 7)); }while(0)

/*
 *  GPIO port mapping utility function
 */
#define GPIO_BASE_TO_PORT_CODE(pGPIOx)  \
    ((pGPIOx == GPIOA) ? 0x00 : \
     (pGPIOx == GPIOB) ? 0x01 : \
     (pGPIOx == GPIOC) ? 0x02 : \
     (pGPIOx == GPIOD) ? 0x03 : \
     (pGPIOx == GPIOF) ? 0x05 : 0xFF) // 0xFF for invalid ports

/*
 * Macors for IRQ(Interrupt Requests) number based on NVIC vector table
 */
#define EXTI0_1_IRQn                  5      /* EXTI Line 0 and 1 interrupts */
#define EXTI2_3_IRQn                  6      /* EXTI Line 2 and 3 interrupts */
#define EXTI4_15_IRQn                 7      /* EXTI Line 4 to 15 interrupts */
/*
 * Generic macros
 */
#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RST 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RST 		RST

#include "gpio.h"


#endif /* INC_STM32G030XX_H_ */























