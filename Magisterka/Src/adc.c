/*
 * adc.c
 *
 *  Created on: 14 cze 2018
 *      Author: Wojtek
 */
#include "adc.h"

uint16_t measured[4] = {0};

uint16_t* GetMeasures()
{
	for(int i = 0 ; i < 4 ; i++)
	{
		measured[i] = performConversion();
	}
	return measured;
}

uint16_t performConversion()
{
	ADC1->CR |= ADC_CR_ADSTART;
	while ((ADC1->ISR & ADC_ISR_EOC) == 0);
	return ADC1->DR;
}

void initializeADC()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	ADC1->SMPR |= ADC_SMPR_SMPR_0 | ADC_SMPR_SMPR_2;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL8 | ADC_CHSELR_CHSEL11;

	ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);
}

