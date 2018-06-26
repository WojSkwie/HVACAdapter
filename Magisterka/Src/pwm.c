/*
 * pwm.c
 *
 *  Created on: 25 cze 2018
 *      Author: Wojtek
 */

#include "pwm.h"

extern TIM_HandleTypeDef htim2;

void initializePWM()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
}

void setOnePWM(uint8_t index, uint16_t value)
{
	__IO uint32_t *pointer = &(TIM2->CCR1);
	pointer[index] = value;
}

void setAllPWM(uint16_t *values)
{
	TIM2->CCR1 = values[0];
	TIM2->CCR2 = values[1];
	TIM2->CCR3 = values[2];
	TIM2->CCR4 = values[3];
}
