/*
 * 005button_interrupt.c
 *
 *  Created on: 2023年2月11日
 *      Author: Lou Yi Cheng
 */



#include "stm32f303xx.h"
#include "string.h"
#define BTN_PRESS 1

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void){
	// LED GPIO configuration
	GPIO_Handle_t Gpio_Led;
	memset(&Gpio_Led,0,sizeof(Gpio_Led));

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
	memset(&Gpio_Btn,0,sizeof(Gpio_Btn));
	Gpio_Btn.pGPIOx = GPIOA;
	Gpio_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Gpio_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	Gpio_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MID;
	Gpio_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

	GPIO_PeriClockControl(Gpio_Btn.pGPIOx, ENABLE);
	GPIO_Init(&Gpio_Btn);

	GPIO_WriteToOutputPin(Gpio_Led.pGPIOx, GPIO_PIN_NO_15, GPIO_PIN_RESET);

	// IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);


	return 0;
}


void EXTI0_IRQHandler(void){
	delay(); //200ms . wait till button de-bouncing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_0); //clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOE, GPIO_PIN_NO_15);
}
