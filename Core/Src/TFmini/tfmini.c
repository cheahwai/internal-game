/*
 * tfmini.c
 *
 *  Created on: 2 May 2022
 *      Author: wai
 */
#include "tfmini.h"

void TFmini_Init(tfmini_t* tfmini, UART_HandleTypeDef* huartx){
	tfmini->huartx = huartx;
	tfmini->checksum = 0;
	HAL_UART_Receive_IT(tfmini->huartx, UARTbuf, 9);
}
//
//void TFmini_set(tfmini_t* tfmini, unsigned char setting, ...){
//	va_list value;
//	va_start(value, setting);
//	tfmini->command = setting;
//	switch(tfmini->command){
//	case FIRMWARE_ver:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x01, 0x5F);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
//		//5A 07 01 V1 V2 V3 SU Version V3.V2.V1
//		break;
//	case TRIGGER_DETECTION:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x04, 0x62);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		After setting the frame rate to 0，detection can be triggered with this 	command
//		break;
//	case FRAME_RATE:
//		tfmini->setbuf = va_arg(value, uint32_t);
//		sprintf(tfmini->buff, "%x%x%x%x%x%x", 0x5A, 0x04, 0x03, tfmini->buf1, tfmini->buf2, tfmini->checksum);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
//		//Frame rate 5A 06 03 LL HH SU
//		//5A 06 03 LL HH SU 1-1000Hz①
//		break;
//	case DATA_FRAME:
//		break;
//	case BAUD_RATE:
//		tfmini->setbuf = va_arg(value, uint32_t);
//		sprintf(tfmini->buff, "%x%x%x%x%x%x%x%x", 0x5A, 0x04, 0x03, tfmini->buf1, tfmini->buf2, tfmini->buf3, tfmini->buf4, tfmini->checksum);
////		5A 08 06 H1 H2 H3 H4 SU Set baud rate③
//		break;
//	case OUTPUT_FORMAT:
//		tfmini->setbuf = va_arg(value, uint32_t);
//		if(tfmini->setbuf == 1)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x05, 0x01, 0x65);
//		if(tfmini->setbuf == 2)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x05, 0x02, 0x66);
//		if(tfmini->setbuf == 3)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x05, 0x06, 0x6A);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
//		UARTPrintString(&huart5, tfmini->buff);
////		5A 05 05 01 65 Standard 9 bytes(cm), default
////		5A 05 05 02 66 Pixhaw
////		5A 05 05 06 6A Standard 9 bytes(mm)
//		break;
//	case OUTPUT_EN:
//		tfmini->setbuf = va_arg(value, uint32_t);
//		if(tfmini->setbuf == 1)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x07, 0x00, 0x66);
//		if(tfmini->setbuf == 2)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x07, 0x01, 0x67);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		5A 05 07 00 66 Disable tfmini->buff output
////		5A 05 07 01 67 Enable tfmini->buff output, default
//		break;
//	case SYSTEM_RESET:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x02, 0x60);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		5A 05 02 00 60 Succeeded
////		5A 05 02 01 61 Failed
//		break;
//	case FACTORY_SET:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x10, 0x6E);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		5A 05 10 00 6E Succeeded
////		5A 05 10 01 6F Failed
//		break;
//	case SAVE:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x11, 0x6F);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		5A 05 11 00 6F Succeeded
////		5A 05 11 01 70 Failed
//		break;
//	}
//	va_end(value);
//}

void TFmini_Handler(tfmini_t* tfmini){
	if(UARTbuf[0] == 0x59 && UARTbuf[1] == 0x59){
		tfmini->checksum = 0;
		tfmini->checksum = UARTbuf[0] + UARTbuf[1] + UARTbuf[2] + UARTbuf[3]
							+ UARTbuf[4] + UARTbuf[5] + UARTbuf[6] + UARTbuf[7];
		if(tfmini->checksum == UARTbuf[8]){
			tfmini->dist = *((int16_t*)&UARTbuf[2]);
			tfmini->dist /= 100;
			tfmini->str  = *((int16_t*)&UARTbuf[4]);
			tfmini->temp = *((int16_t*)&UARTbuf[6]) / 8 - 256;
		}
		memset(UARTbuf, 0, 9);
	}

//	HAL_UART_Receive_IT(tfmini->huartx, UARTbuf, 9);
}
