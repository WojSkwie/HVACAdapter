/*
 * adc.h
 *
 *  Created on: 14 cze 2018
 *      Author: Wojtek
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32l0xx_hal.h"
#include "stdint.h"
#include "stdio.h"


uint16_t* GetMeasures();
uint16_t performConversion();

#endif /* ADC_H_ */
