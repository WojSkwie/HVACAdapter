/*
 * comm.h
 *
 *  Created on: 12 cze 2018
 *      Author: Wojtek
 */

#ifndef COMM_H_
#define COMM_H_

#include "stm32l0xx_hal.h"
#include "stdint.h"
#include "stdio.h"

void initializeReceive();
void sendWholeData();
void sendSingleData(uint8_t index, uint16_t data);
uint16_t getSingleOutput(uint8_t index);
uint16_t parseSingleValueFromFrame(uint8_t frame[]);
void getAllValuesFromFrame(uint8_t frame[], uint16_t values[]);
void parseFrame();
uint8_t crc8(uint8_t * data, uint16_t size);

#endif /* COMM_H_ */
