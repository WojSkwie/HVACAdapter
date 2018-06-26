/*
 * gpio.h
 *
 *  Created on: 16 cze 2018
 *      Author: Wojtek
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32l0xx_hal.h"
#include "stdint.h"
#include "stdio.h"

void initializeADCPins();
void initializeDigitalPins();
uint8_t readDigital();
uint8_t readOneDigital(uint8_t index);
void writeDigital(uint8_t outputs);
void writeOneDigital(uint8_t output, uint8_t index);

#endif /* GPIO_H_ */
