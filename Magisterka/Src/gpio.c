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

void initializeDigitalPins()
{
	GPIOC->MODER &= ~(GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1
			| GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
	GPIOC->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12
			| GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE15);
}

uint8_t readDigital()
{
	uint8_t digitalInputs = (GPIOC->IDR >> 10) & 0b00111111;
	return ~(digitalInputs);
}

uint8_t readOneDigital(uint8_t index)
{
	uint8_t input = ((GPIOC->IDR >> (10 + index)) & 0x1);
	return input;
}
void writeDigital(uint8_t outputs)
{
	GPIOC->ODR |= ((outputs & 0x3F) << 4);
	GPIOC->ODR &= ((~(~outputs & 0x3F)) << 4);
}
