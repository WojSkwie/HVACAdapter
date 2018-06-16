/*
 * gpio.c
 *
 *  Created on: 16 cze 2018
 *      Author: Wojtek
 */

#include "gpio.h"

void initializeADCPins()
{
	GPIOA->MODER |= GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7;
}
