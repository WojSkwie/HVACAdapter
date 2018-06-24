/*
 * gpio.c
 *
 *  Created on: 16 cze 2018
 *      Author: Wojtek
 */

#include "gpio.h"

void initializeADCPins()
{
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3;
	GPIOB->MODER |= GPIO_MODER_MODE0;
	GPIOC->MODER |= GPIO_MODER_MODE1;
}
