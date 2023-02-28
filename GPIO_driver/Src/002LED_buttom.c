/*
 * 001LED_toogle.c
 *
 *  Created on: 2023年2月8日
 *      Author: Lou Yi Cheng
 */


#include "stm32f303xx.h"
#define BTN_PRESS 1

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void){
	// LED GPIO configuration
	GPIO_Handle_t Gpio_Led;
	Gpio_Led.pGPIOx = GPIOE;
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MID;
	Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(Gpio_Led.pGPIOx, ENABLE);
	GPIO_Init(&Gpio_Led);

	// this is BTN GPIO configuration
	GPIO_Handle_t Gpio_Btn;
	Gpio_Btn.pGPIOx = GPIOA;
	Gpio_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Gpio_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Gpio_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MID;
//	Gpio_Btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpio_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(Gpio_Btn.pGPIOx, ENABLE);
	GPIO_Init(&Gpio_Btn);


	while(1){
		if(GPIO_ReadFromInputPin(Gpio_Btn.pGPIOx, GPIO_PIN_NO_0) == BTN_PRESS){
			delay();
			GPIO_ToggleOutputPin(Gpio_Led.pGPIOx, GPIO_PIN_NO_15);
		}
	}
	return 0;
}
