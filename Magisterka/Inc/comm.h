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

void disableHalfTransferIT();
void initializeReceive();
void sendAllInputValues(uint16_t* analogValues, uint8_t digitalValues);
void sendAnalogValue(uint8_t index, uint16_t value);
void sendDigitalValue(uint8_t index, uint8_t value);
uint16_t getAnalogValueFromFrame(uint8_t frame[]);
uint8_t getDigitalValueFromFrame(uint8_t frame[]);
void getAllValuesFromFrame(uint8_t frame[], uint16_t analogValues[], uint8_t* digitalValues);
void parseFrame();
uint8_t crc8(uint8_t * data, uint16_t size);

#endif /* COMM_H_ */
