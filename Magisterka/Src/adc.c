/*
 * adc.c
 *
 *  Created on: 14 cze 2018
 *      Author: Wojtek
 */
#include "adc.h"

uint16_t measured[4] = {0};
//extern ADC_HandleTypeDef hadc;

uint16_t* GetMeasures()
{
	/*ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);*/

	for(int i = 0 ; i < 4 ; i++)
	{
		measured[i] = performConversion();
	}
	//measured[0] = performConversion();
	return measured;
}

uint16_t performConversion()
{
	ADC1->CR |= ADC_CR_ADSTART;
	while ((ADC1->ISR & ADC_ISR_EOC) == 0);
	return ADC1->DR;
	/*HAL_ADC_Start(&hadc);
	//while(!ADC1->ISR & ADC_ISR_EOC);
	HAL_ADC_PollForConversion(&hadc, 10);
	if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
	{
		return HAL_ADC_GetValue(&hadc);
	}
	else*/
	{
		return 0;
	}
}

void initializeADC()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	ADC1->SMPR |= ADC_SMPR_SMPR_0 | ADC_SMPR_SMPR_1;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL7;

	ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);
}

