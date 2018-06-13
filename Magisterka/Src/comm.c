/*
 * comm.c
 *
 *  Created on: 12 cze 2018
 *      Author: Wojtek
 */
#define frameSize 12
#include "comm.h"
#include "string.h"

uint16_t outputs[4] = {0};
uint16_t inputs[4] = {0};
const uint8_t startByte = 0xFE;
const uint8_t endByte = 0xF0;
extern UART_HandleTypeDef huart1;
uint8_t receivedData[20] = {0};

enum cmd
{
	writeAll = 0x1,
	writeOne = 0x2,
	readAll = 0x11,
	readOne = 0x12,
	AnswerAll = 0x21,
	AnswerOne = 0x22
};

void initializeReceive()
{
	HAL_UART_Receive_DMA(&huart1, receivedData, frameSize);
}

void sendWholeData()
{
	uint8_t frame[frameSize] = {0};
	frame[0] = startByte;
	frame[1] = AnswerAll;
	memcpy(&frame[2], inputs,8);
	uint8_t crc = crc8(frame,10);
	frame[10] = crc;
	frame[11] = endByte;
	HAL_UART_Transmit_IT(&huart1,frame,12);
}

void sendOneData(uint8_t index)
{
	uint8_t frame[frameSize] = {0};
	frame[0] = startByte;
	frame[1] = AnswerOne;
	frame[2] = index;
	uint16_t temp = inputs[index];
	frame[3] = (uint8_t)((temp >> 8) & 0xFF);
	frame[4] = (uint8_t)(temp & 0xFF);
	uint8_t crc = crc8(frame,10);
	frame[10] = crc;
	frame[11] = endByte;
	HAL_UART_Transmit_IT(&huart1,frame,12);
}

uint16_t getSingleValueFromFrame(uint8_t frame[])
{
	uint16_t value = (frame[3] << 8) + frame[4];
	return value;
}

void getAllValuesFromFrame(uint8_t frame[], uint16_t values[])
{
	for(int i = 0 ; i < 4 ; i++)
	{
		values[i] = (frame[3 + 2 * i] << 8) + frame[4 + 2 * i];
	}
}


void parseFrame()
{
	if(receivedData[0] == startByte && receivedData[11] == endByte)
	{
		uint8_t crc8hw = crc8(receivedData, 10);
		if(crc8hw != receivedData[10]) return;
		switch(receivedData[1])
		{
			case writeAll:
			{
				uint16_t values[4] = {0};
				getAllValuesFromFrame(receivedData, values);
				break;
			}
			case writeOne:
			{
				uint16_t singleValue = getSingleValueFromFrame(receivedData);
				break;
			}
			case readAll:
				sendWholeData();
				break;
			case readOne:
				sendOneData(receivedData[11]);
				break;
		}
	}
	else
	{

	}
}

uint8_t crc8(uint8_t * data, uint16_t size)
{
    uint8_t crc = 0;
    for ( uint16_t i = 0; i < size; ++i )
    {
    	uint8_t inbyte = data[i];
        for ( uint8_t j = 0; j < 8; ++j )
        {
        	uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if ( mix ) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}
