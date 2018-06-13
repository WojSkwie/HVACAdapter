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

uint8_t crc8(uint8_t * data, uint16_t size);
void sendWholeData();
void sendOneData(uint8_t index);
void initializeReceive();
void parseFrame();

#endif /* COMM_H_ */
