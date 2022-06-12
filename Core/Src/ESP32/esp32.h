/*
 * esp32.h
 *
 *  Created on: Mar 25, 2022
 *      Author: wai
 */

#ifndef SRC_ESP32_ESP32_H_
#define SRC_ESP32_ESP32_H_

#include "../BIOS/bios.h"

struct{
	UART_HandleTypeDef* huart;
	uint8_t buffer[10];
	char lagoriColor;
	uint16_t lagoriHeight;
	uint16_t lagoriWidth;
	uint32_t lagoriArea;
	uint8_t sample_count;
	uint8_t sample[20];
	uint8_t data_ready;
}esp32;

void esp32_Init(UART_HandleTypeDef* huart);
void esp32StartSend(void);
void esp32StopSend(void);
void esp32Handler(void);
int esp32ProcData (void);

#endif /* SRC_ESP32_ESP32_H_ */
