/*
 * pwm.h
 *
 *  Created on: 25 cze 2018
 *      Author: Wojtek
 */

#ifndef PWM_H_
#define PWM_H_

#include "stm32l0xx_hal.h"
#include "stdint.h"
#include "stdio.h"

void initializePWM();
void setOnePWM(uint8_t index, uint16_t value);
void setAllPWM(uint16_t *values);

#endif /* PWM_H_ */
