/*
 * esp32.c
 *
 *  Created on: Mar 25, 2022
 *      Author: wai
 */
#include "esp32.h"

void esp32_Init(UART_HandleTypeDef* huart){
	esp32.huart = huart;
	esp32.sample_count=0;
	esp32.data_ready=0;
	HAL_UART_Receive_IT(huart, esp32.buffer, 1);
}

void esp32StartSend(void){
	char espBuff[1] = "c";
//	UARTPrintString(esp32.huart, espBuff);

}

void esp32StopSend(void){
	char espBuff[1] = "s";
//	UARTPrintString(esp32.huart, espBuff);
}

void esp32Handler(void){
	static uint8_t state=0;
	switch(state){
	case 0:
		if(esp32.buffer[0] == 0x03){
			state = 1;
			HAL_UART_Receive_IT(esp32.huart, esp32.buffer, 3);
		}else{
			HAL_UART_Receive_IT(esp32.huart, esp32.buffer, 1);
		}
		break;
	case 1:
		if(esp32.buffer[2] == 0x24){
			esp32.sample[esp32.sample_count++] = esp32.buffer[0];
			if(esp32.sample_count>20){
				esp32StopSend();
				esp32.data_ready=1;
				esp32.sample_count=0;
			}
		}
		state = 0;
		HAL_UART_Receive_IT(esp32.huart, esp32.buffer, 1);
		break;
	}
}

int esp32ProcData (void){

	int one=0, two=0, three=0, four=0, five=0, six=0;


	for(uint8_t i=0; i<esp32.sample_count; i++){
		switch(esp32.sample[i]){

		case 1:
			one++;
			break;

		case 2:
			two++;
			break;

		case 3:
			three++;
			break;

		case 4:
			four++;
			break;

		case 5:
			five++;
			break;

		case 6:
			six++;
			break;

		}

	}

	if(one>=two&&one>=three&&one>=four&&one>=five&&one>=six)
		return 1;

	else if(two>=three&&two>=four&&two>=five&&two>=six)
		return 2;

	else if(three>=four&&three>=five&&three>six)
		return 3;

	else if(four>=five&&four>=six)
		return 4;

	else if (five>=six)
		return 5;

	else
		return 6;

}

