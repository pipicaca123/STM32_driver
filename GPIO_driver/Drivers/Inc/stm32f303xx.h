/*
 * stm32f303xx.h
 *
 *  Created on: Feb 6, 2023
 *      Author: Lou Yi Cheng
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_

#include <stdint.h>
#define __weak __attribute__((weak))

//some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET 			SET


/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 * more detail in Cortex-M4 Device Generic User Guide
 */
#define NVIC_ISER0          		((volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1          		((volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2         			((volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3         			((volatile uint32_t*)0xE000E10c )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 * more detail in Cortex-M4 Device Generic User Guide
 */
#define NVIC_ICER0 					((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1					((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  				((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3					((volatile uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 			((volatile uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * Base address of Flash and SRAM memories
 */
#define FLASH_BASEADDR				0x08000000U 	/* main memory */
#define SRAM_BASEADDR				0x20000000U
#define CCM_SRAM_BASEADDR			0x10000000U

/*
 * AHBx and APBx Bus Peripheral base addresses
 * TODO : Complete for all other peripherals
 */
#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x48000000U

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */
#define GPIOA_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB2PERIPH_BASEADDR + 0x1400)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */
#define RCC_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define SYSCFG_BASEADDR 			(APB2PERIPH_BASEADDR + 0x0000)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x0400)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)


/**********************************peripheral register definition structures **********************************/
typedef struct
{
	volatile uint32_t MODER; 	// offset: 0x00
	volatile uint32_t OTYPER; 	// offset: 0x04
	volatile uint32_t OSPEEDR; 	// offset: 0x08
	volatile uint32_t PUPDR;	// offset: 0x0C
	volatile uint32_t IDR;		// offset: 0x10
	volatile uint32_t ODR;		// offset: 0x14
	volatile uint32_t BSRR;		// offset: 0x18
	volatile uint32_t LCKR;		// offset: 0x1C
	volatile uint32_t AFR[2];	// offset: (L)0x20 (H) 0x24
	volatile uint32_t BRR;		// offset: 0x28
}GPIO_RegDef_t;


typedef struct
{
	volatile uint32_t CR; 		// offset: 0x00
	volatile uint32_t CFGR; 	// offset: 0x04
	volatile uint32_t CIR; 		// offset: 0x08
	volatile uint32_t APB2RSTR; // offset: 0x0C
	volatile uint32_t APB1RSTR; // offset: 0x10
	volatile uint32_t AHBENR; 	// offset: 0x14
	volatile uint32_t APB2ENR;  // offset: 0x18
	volatile uint32_t APB1ENR;  // offset: 0x1C
	volatile uint32_t BDCR;	    // offset: 0x20
	volatile uint32_t CSR;  	// offset: 0x24
	volatile uint32_t AHBRSTR;  // offset: 0x28
	volatile uint32_t CFGR2; 	// offset: 0x2C
	volatile uint32_t CFGR3; 	// offset: 0x30
}RCC_RegDef_t;

typedef struct
{
	volatile uint32_t IMR1; 		// offset: 0x00
	volatile uint32_t EMR1; 		// offset: 0x04
	volatile uint32_t RTSR1; 		// offset: 0x08
	volatile uint32_t FTSR1; 		// offset: 0x0C
	volatile uint32_t SWIER1; 		// offset: 0x10
	volatile uint32_t PR1; 			// offset: 0x14
	volatile uint32_t RESERVED[2]; 	// offset: 0x18,0x1C
	volatile uint32_t IMR2; 		// offset: 0x20
	volatile uint32_t EMR2; 		// offset: 0x24
	volatile uint32_t RTSR2; 		// offset: 0x28
	volatile uint32_t FTSR2; 		// offset: 0x2C
	volatile uint32_t SWIER2; 		// offset: 0x30
	volatile uint32_t PR2; 			// offset: 0x34
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t CFCR1; 		// offset: 0x00
	volatile uint32_t RCR; 			// offset: 0x04
	volatile uint32_t EXTICR[4]; 	// offset: 0x08 ~ 0x14
	volatile uint32_t CFGR2; 		// offset: 0x18
	uint32_t RESERVED[11]; 			// offset 0x1C ~ 0x44
	volatile uint32_t CFGR4; 		// offset: 0x48
	uint32_t RESERVED2; 			// offset 0x4C
	volatile uint32_t CFGR3; 		// offset: 0x50
}SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	volatile uint32_t CR1;			// offset: 0x00
	volatile uint32_t CR2;			// offset: 0x04
	volatile uint32_t SR;			// offset: 0x08
	volatile uint32_t DR;			// offset: 0x0C
	volatile uint32_t CRCPR;		// offset: 0x10
	volatile uint32_t RXCRCR;		// offset: 0x14
	volatile uint32_t TXCRCR;		// offset: 0x18
	volatile uint32_t I2SCFGR;		// offset: 0x1C
	volatile uint32_t I2SPR;		// offset: 0x20
} SPI_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA 				((GPIO_RegDef_t*) 	GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t*) 	GPIOB_BASEADDR)
#define GPIOC 				((GPIO_RegDef_t*) 	GPIOC_BASEADDR)
#define GPIOD 				((GPIO_RegDef_t*) 	GPIOD_BASEADDR)
#define GPIOE 				((GPIO_RegDef_t*) 	GPIOE_BASEADDR)
#define GPIOF 				((GPIO_RegDef_t*) 	GPIOF_BASEADDR)

#define RCC					((RCC_RegDef_t*) 	RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)	EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)  	SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)  	SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)  	SPI3_BASEADDR)
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()		(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()		(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()		(RCC->AHBENR |= (1 << 20))
#define GPIOE_PCLK_EN()		(RCC->AHBENR |= (1 << 21))
#define GPIOF_PCLK_EN()		(RCC->AHBENR |= (1 << 22))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() 		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 22))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() 		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() 		(RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN() 	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN() 	(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN() 	(RCC->APB1ENR |= (1 << 20))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1 << 0))


/*
 * Clock Enable Macros for SPIx peripheral
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()  			do{(RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17));}while(0)
#define GPIOB_REG_RESET()  			do{(RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18));}while(0)
#define GPIOC_REG_RESET()  			do{(RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19));}while(0)
#define GPIOD_REG_RESET()  			do{(RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20));}while(0)
#define GPIOE_REG_RESET()  			do{(RCC->AHBRSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21));}while(0)
#define GPIOF_REG_RESET()  			do{(RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22));}while(0)

/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:0)

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()		(RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()		(RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()		(RCC->AHBENR &= ~(1 << 20))
#define GPIOE_PCLK_DI()		(RCC->AHBENR &= ~(1 << 21))
#define GPIOF_PCLK_DI()		(RCC->AHBENR &= ~(1 << 22))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 15))

/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()  			do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()  			do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()  			do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR 			3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST 	7
#define SPI_CR1_SSI		 	8
#define SPI_CR1_SSM	 		9
#define SPI_CR1_RXONLY	 	10
#define SPI_CR1_CRCL 		11 // DLL: name as CRCL bit in stm32f303xx
#define SPI_CR1_CRCNEXT 	12
#define SPI_CR1_CRCEN 		13
#define SPI_CR1_BIDIOE 		14
#define SPI_CR1_BIDIMODE 	15

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRLVL		9
#define SPI_SR_FTLVL		11

/*
 * IRQ(Interrupt Request) Numbers of STM32F303x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2_TS 	8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_I2C2_EV     	33
#define IRQ_NO_I2C2_ER      34
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

#include "stm32f303xx_GPIO_driver.h"
#include "stm32f303xx_SPI.h"
#endif /* INC_STM32F303XX_H_ */
