/*
 * comm.c
 *
 *  Created on: 12 cze 2018
 *      Author: Wojtek
 */
#define frameSize 13
#include "comm.h"
#include "string.h"
#include "adc.h"
#include "pwm.h"

const uint8_t startByte = 0xFE;
const uint8_t endByte = 0xF0;
//extern UART_HandleTypeDef huart1;
uint8_t receivedData[20] = {0};

enum cmd
{
	writeAll = 0x1,
	writeOneAn = 0x2,
	writeOneDi = 0x3,
	readAll = 0x11,
	readOneAn = 0x12,
	readOneDi = 0x13,
	answerAll = 0x21,
	answerOneAn = 0x22,
	answerOneDi = 0x23,
	echo = 0xaa,
	echoAnswer = 0x55,
};

void disableHalfTransferIT()
{
	DMA1_Channel3->CCR &= ~(DMA_CCR_HTIE);
}

void initializeReceive()
{
	DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
	DMA1->IFCR |= DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3;
	DMA1_Channel3->CNDTR = frameSize;
	DMA1_Channel3->CPAR = (uint32_t) (&(USART1->RDR));
	DMA1_Channel3->CMAR = (uint32_t)(receivedData);
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	//HAL_UART_Receive_DMA(&huart1, receivedData, frameSize);
	//disableHalfTransferIT();
}

void sendAllInputValues(uint16_t* analogValues, uint8_t digitalValues)
{
	uint8_t frame[frameSize] = {0};
	frame[0] = startByte;
	frame[1] = answerAll;
	for(int i = 0 ; i < 4 ; i++)
	{
		frame[2+2*i] = (uint8_t)((analogValues[i] >> 8) & 0xFF);
		frame[3+2*i] = (uint8_t)(analogValues[i] & 0xFF);
	}
	frame[10] = digitalValues;
	uint8_t crc = crc8(&frame[1],frameSize - 3);
	frame[frameSize-2] = crc;
	frame[frameSize-1] = endByte;
	sendFrameDMA(frame);
	//HAL_UART_Transmit_IT(&huart1, frame, frameSize); TODO
}

void sendAnswerToEcho()
{
	uint8_t frame[frameSize] = {0};
	frame[0] = startByte;
	frame[1] = echoAnswer;
	uint8_t crc = crc8(&frame[1],frameSize - 3);
	frame[frameSize-2] = crc;
	frame[frameSize-1] = endByte;
	sendFrameDMA(frame);
	//HAL_UART_Transmit_IT(&huart1, frame, frameSize); TODO
}

void sendAnalogValue(uint8_t index, uint16_t value)
{
	uint8_t frame[frameSize] = {0};
	frame[0] = startByte;
	frame[1] = answerOneAn;
	frame[2] = index;
	frame[3] = (uint8_t)((value >> 8) & 0xFF);
	frame[4] = (uint8_t)(value & 0xFF);
	uint8_t crc = crc8(&frame[1],frameSize - 3);
	frame[frameSize-2] = crc;
	frame[frameSize-1] = endByte;
	sendFrameDMA(frame);
	//HAL_UART_Transmit_IT(&huart1, frame, frameSize); TODO
}

void sendDigitalValue(uint8_t index, uint8_t value)
{
	uint8_t frame[frameSize] = {0};
	frame[0] = startByte;
	frame[1] = answerOneDi;
	frame[2] = index;
	frame[3] = value;
	uint8_t crc = crc8(&frame[1],frameSize - 3);
	frame[frameSize-2] = crc;
	frame[frameSize-1] = endByte;
	sendFrameDMA(frame);
	//HAL_UART_Transmit_IT(&huart1, frame, frameSize); TODO
}

uint16_t getAnalogValueFromFrame(uint8_t frame[])
{
	uint16_t value = (frame[3] << 8) + frame[4];
	return value;
}

uint8_t getDigitalValueFromFrame(uint8_t frame[])
{
	return frame[3];
}

void getAllValuesFromFrame(uint8_t frame[], uint16_t analogValues[], uint8_t* digitalValues)
{
	for(int i = 0 ; i < 4 ; i++)
	{
		analogValues[i] = (frame[2 + 2 * i] << 8) + frame[3 + 2 * i];
	}
	digitalValues[0] = frame[10];
}


void parseFrame()
{
	if(receivedData[0] == startByte && receivedData[frameSize-1] == endByte)
	{
		uint8_t crc8hw = crc8(&receivedData[1], frameSize - 3);
		if(crc8hw != receivedData[frameSize-2]) return;
		switch(receivedData[1])
		{
			case writeAll:
			{
				uint16_t analogValues[4] = {0};
				uint8_t digitalValues = 0;
				getAllValuesFromFrame(receivedData, analogValues, &digitalValues);
				setAllPWM(analogValues);
				writeDigital(digitalValues);
				break;
			}
			case writeOneAn:
			{
				uint8_t index = receivedData[2];
				uint16_t singleValue = getAnalogValueFromFrame(receivedData);
				setOnePWM(index, singleValue);
				break;
			}
			case writeOneDi:
			{
				uint8_t index = receivedData[2];
				uint8_t value = getDigitalValueFromFrame(receivedData);
				writeOneDigital(value, index);
				break;
			}
			case readAll:
			{
				uint16_t* adc = GetMeasures();
				uint8_t digital = readDigital();
				sendAllInputValues(adc, digital);
				break;
			}
			case readOneAn:
			{
				uint16_t* adc = GetMeasures();
				uint8_t index = receivedData[2];
				sendAnalogValue(index, adc[index]);
				break;
			}
			case readOneDi:
			{
				uint8_t index = receivedData[2];
				uint8_t digital = readOneDigital(index);
				sendDigitalValue(index, digital);
				break;
			}
			case echo:
				sendAnswerToEcho();
				break;
		}
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

void initializeUART()
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_TE |USART_CR1_RE |USART_CR1_UE;
	USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; //USART1->RQR |= RXFRQ:
	USART1->BRR = 139;
	NVIC_EnableIRQ(USART1_IRQn);
}

void initializeDMA()
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_CSELR->CSELR |= (3 << DMA_CSELR_C2S_Pos) | (3 << DMA_CSELR_C3S_Pos);

	DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_TEIE | DMA_CCR_TCIE;
	DMA1_Channel3->CNDTR = frameSize;
	DMA1_Channel3->CPAR = (uint32_t) (&(USART1->RDR));
	DMA1_Channel3->CMAR = (uint32_t)(receivedData);

	DMA1_Channel3->CCR |= DMA_CCR_EN;
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	//DMA1->I
}

void USART1_IRQHandler(void)
{
	if(USART1->ISR & USART_ISR_RXNE)
	{
		uint8_t received = USART1->RDR;
		USART1->TDR = received;
	}
}

void DMA1_Channel2_3_IRQHandler(void)
{
	if(DMA1->ISR & DMA_ISR_TCIF3)
	{
		parseFrame();
		//initializeReceive();
	}
	else if (DMA1->ISR & DMA_ISR_TEIF3)
	{
		uint8_t status = DMA1->ISR;
	}
	initializeReceive();
}

void sendFrameDMA(uint8_t *frame)
{
	DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
	DMA1->IFCR |= DMA_IFCR_CTCIF2 |DMA_IFCR_CTEIF2;
	DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel2->CNDTR = frameSize;
	DMA1_Channel2->CPAR = (uint32_t) (&(USART1->TDR));
	DMA1_Channel2->CMAR = (uint32_t) frame;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
}

void sendByte()
{
	USART1->TDR = 1;
}
