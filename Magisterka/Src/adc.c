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
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_6;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

	measured[0] = performConversion();
	return measured;
}

uint16_t performConversion()
{
	HAL_ADC_Start(&hadc);
	//while(!ADC1->ISR & ADC_ISR_EOC);
	HAL_ADC_PollForConversion(&hadc, 1);
	if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
	{
		return HAL_ADC_GetValue(&hadc);
	}
	else
	{
		return 0;
	}
}

