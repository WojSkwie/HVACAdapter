/*
 * adc.c
 *
 *  Created on: 14 cze 2018
 *      Author: Wojtek
 */
#include "adc.h"

uint16_t measured[4] = {0};
extern ADC_HandleTypeDef hadc;

uint16_t* GetMeasures()
{
	measured[0] = performConversion();
	return measured;
}

uint16_t performConversion()
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
	{
		return HAL_ADC_GetValue(&hadc);
	}
	else
	{
		return 0;
	}
}

