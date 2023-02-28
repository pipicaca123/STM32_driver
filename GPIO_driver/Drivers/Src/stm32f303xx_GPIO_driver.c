/*
 * stm32f303xx_GPIO.c
 *
 *  Created on: 2023年2月7日
 *      Author: Lou Yi Cheng
 */


#include "stm32f303xx_GPIO_driver.h"

// enable or disable PeriClock
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;
	// enable the peripheral clock (before setting PERI. ,
	// PERI. clock should set up first.)
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configure the mode of gpio pin
	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		 temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // read user setup
		 pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
		 pGPIOHandle->pGPIOx->MODER |= temp; // setting
	 }
	 else{ // interrupt mode

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// 1. configure the FTSR & RTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // symmetric with GPIO pin number
			// Clear the corresponding RTSR bit
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// 1. configure the FTSR & RTSR
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // symmetric with GPIO pin number
			// Clear the corresponding RTSR bit
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// 1. configure the FTSR & RTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // symmetric with GPIO pin number
			// Clear the corresponding RTSR bit
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);
		//3 . enable the exti interrupt delivery using IMR
		EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	 }

	 //2. configure the speed
	 temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	 pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	 pGPIOHandle->pGPIOx->OSPEEDR |= temp; // setting

	 //3. configure the pupd settings
	 temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	 pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	 pGPIOHandle->pGPIOx->PUPDR |= temp; // setting
	 //4. configure the optype
	 //3. configure the pupd settings
	 temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	 pGPIOHandle->pGPIOx->OTYPER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	 pGPIOHandle->pGPIOx->OTYPER |= temp; // setting
	 //5. configure the alt functionality
	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN){
		//configure the alt function registers.
		uint8_t temp1, temp2;
		temp =  pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (temp << (4 * temp2));
	 }
}

void GPIO_GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001) ;
	return value;

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	return (uint16_t)(pGPIOx->IDR);
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	// write 1 to the output data register at the bit field corresponding to the pin number
	if(Value == GPIO_PIN_SET)
		pGPIOx->ODR |= (1 << PinNumber);

	//write 0
	else if(Value == GPIO_PIN_RESET)
		pGPIOx->ODR &= ~(1 << PinNumber);
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value; // write the value directly to GPIOx
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber < 63 && IRQNumber > 31 ){
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber <96 && IRQNumber >= 64 ){
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}
	else if(EnorDi == DISABLE){
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber < 63 && IRQNumber > 31 ){
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber <96 && IRQNumber >= 64 ){
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); // in PRI reg., only high 4 bits are implemented
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR1 & (1 << PinNumber)){
		EXTI->PR1 |= (1 << PinNumber); //clear
	}
}
