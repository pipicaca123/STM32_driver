/*
 * 001LED_toogle.c
 *
 *  Created on: 2023年2月8日
 *      Author: Lou Yi Cheng
 */


#include "stm32f303xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void){
	GPIO_Handle_t Gpio_Led;
	Gpio_Led.pGPIOx = GPIOE;
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MID;
	Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(Gpio_Led.pGPIOx, ENABLE);
	GPIO_Init(&Gpio_Led);
	while(1){
		GPIO_ToggleOutputPin(Gpio_Led.pGPIOx, GPIO_PIN_NO_15);
		delay();
	}
	return 0;
}
