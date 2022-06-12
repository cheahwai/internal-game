/*
 * TuningInterface.c
 *
 *  Created on: Mar 18, 2022
 *      Author: lokcharming
 */
#include "TuningInterface.h"

static int numInt[TOTAL_PAGE] = {
		NUM_INT_TUNE_LIST0,
		NUM_INT_TUNE_LIST1,
		NUM_INT_TUNE_LIST2,
		NUM_INT_TUNE_LIST3,
		NUM_INT_TUNE_LIST4
};

static int numFloat[TOTAL_PAGE] = {
		NUM_FLOAT_TUNE_LIST0,
		NUM_FLOAT_TUNE_LIST1,
		NUM_FLOAT_TUNE_LIST2,
		NUM_FLOAT_TUNE_LIST3,
		NUM_FLOAT_TUNE_LIST4
};

static Tune_Int_t* intList[TOTAL_PAGE] = {
		TuneIntList0,
		TuneIntList1,
		TuneIntList2,
		TuneIntList3,
		TuneIntList4
};

static Tune_Float_t* floatList[TOTAL_PAGE] = {
		TuneFloatList0,
		TuneFloatList1,
		TuneFloatList2,
		TuneFloatList3,
		TuneFloatList4
};

static uint32_t intAddress[TOTAL_PAGE]={
		SECTOR_6_FOR_PAGE_0,
		SECTOR_6_FOR_PAGE_1,
		SECTOR_6_FOR_PAGE_2,
		SECTOR_6_FOR_PAGE_3,
		SECTOR_6_FOR_PAGE_4
};

static uint32_t floatAddress[TOTAL_PAGE]={
		SECTOR_7_FOR_PAGE_0,
		SECTOR_7_FOR_PAGE_1,
		SECTOR_7_FOR_PAGE_2,
		SECTOR_7_FOR_PAGE_3,
		SECTOR_7_FOR_PAGE_4
};

void tuneRegister(int numPage, UART_HandleTypeDef* huartx){
	tunePage = numPage;
	tuneUart = huartx;

	for(int i=0; i<tunePage; i++){
		tuneWriteSuccess += numInt[i];
		tuneWriteSuccess += numFloat[i];
	}
}

void tunePending(void){
	HAL_UART_Receive_DMA(tuneUart, (uint8_t *)&tuneCommand, 1);
}

void tuneInit(TUNE_PAGE page){
	//[NUM_INT_TUNE][NUM_FLOAT_TUNE]
	//[0x52][0x01][strlen][str...][*curValue...][min ...][max ...][0x31][0x40] format of INT
	//[0x14][0x01][strlen][str...][*curValue...][minF...][maxF...][0x20][0x00] format of FLOAT


	uint8_t numVar[2] = {numInt[page], numFloat[page]};

	HAL_UART_Transmit(tuneUart, numVar, 2, 10);
	uint8_t sendBuf[100];
	for(int i=0; i<numInt[page]; i++){
		sendBuf[0] = 0x52;	// '4'
		sendBuf[1] = 0x01;	//Start of Header
		uint8_t strLen = strlen(intList[page][i].varName);
		sendBuf[2] = strLen;
		uint8_t len = 1+1+1+strLen+4+4+4+1+1;

		sendBuf[len-2] = 0x31; sendBuf[len-1] = 0x40;		//Unit seperator, '(';
		memcpy(&sendBuf[3], intList[page][i].varName, strLen);
		memcpy(&sendBuf[3+strLen], intList[page][i].ptr, 4);
		memcpy(&sendBuf[3+strLen+4], &intList[page][i].min, 4);
		memcpy(&sendBuf[3+strLen+8], &intList[page][i].max, 4);
		HAL_UART_Transmit(tuneUart, sendBuf, len, 1000);
	}

	for(int i=0; i<numFloat[page]; i++){
		sendBuf[0] = 0x14;
		sendBuf[1] = 0x01;
		uint8_t strLen = strlen(floatList[page][i].varName);
		sendBuf[2] = strLen;
		uint8_t len = 3+strLen+4+4+4+1+1;

		sendBuf[len-2] = 0x20; sendBuf[len-1] = 0x00;
		memcpy(&sendBuf[3], floatList[page][i].varName, strLen);
		memcpy(&sendBuf[3+strLen], floatList[page][i].ptr, 4);
		memcpy(&sendBuf[3+strLen+4], &floatList[page][i].min, 4);
		memcpy(&sendBuf[3+strLen+8], &floatList[page][i].max, 4);

		HAL_UART_Transmit(tuneUart, sendBuf, len, 100);
	}
	tunePending();
}

void tuneUpdate(TUNE_PAGE page){
	//[0x52][0x01][index][*curValue...][0x31][0x40] format of INT
	//[0x14][0x01][index][*curValue...][0x20][0x00] format of FLOAT

	for(int i=0; i<numInt[page]; i++){
		int len = 1+1+1+4+1+1;
		uint8_t sendBuf[len];
		sendBuf[0] = 0x52; sendBuf[1] = 0x01; sendBuf[len-2] = 0x31; sendBuf[len-1] = 0x40;
		sendBuf[2] = i;
		memcpy(&sendBuf[3], intList[page][i].ptr, 4);
		HAL_UART_Transmit(tuneUart, sendBuf, len, 100);
	}

	for(int i=0; i<numFloat[page]; i++){
		int len = 1+1+1+4+1+1;
		uint8_t sendBuf[len];
		sendBuf[0] = 0x14; sendBuf[1] = 0x01; sendBuf[len-2] = 0x20; sendBuf[len-1] = 0x00;
		sendBuf[2] = i;
		memcpy(&sendBuf[3], floatList[page][i].ptr, 4);
		HAL_UART_Transmit(tuneUart, sendBuf, len, 100);
	}

	tunePending();
}

void tuneEdit(TUNE_PAGE page, RNS_interface_t* rns){
	//[0x52][0x01][index][editValue...][0x31][0x40] format of INT
	//[0x14][0x01][index][editValue...][0x20][0x00] format of FLOAT
	//[0x88][0x77] terminate, back to tune pending
	static uint8_t state = 0;

	switch(state){
	case 0://When first received tuneCommand == 'e'
		HAL_UART_Receive_DMA(tuneUart, &tuneHeader, 1);
		state = 1;
		break;
	case 1:
		if(tuneHeader == 0x52 || tuneHeader == 0x14 || tuneHeader == 0x88){
			state = 2;
			HAL_UART_Receive_DMA(tuneUart, &tuneHeader1, 1);
		}
		else{
			HAL_UART_Receive_DMA(tuneUart, &tuneHeader, 1);
			state = 1;
		}
		break;
	case 2:
		if(tuneHeader1 == 0x01){
			HAL_UART_Receive_DMA(tuneUart, tuneBuffer, 7);
			state = 3;
		}
		else if(tuneHeader1 == 0x77){
			if(page == 0){
				RNSSet(rns, RNS_F_LEFT_VEL_PID,  *(floatList[page][0].ptr), *(floatList[page][1].ptr), *(floatList[page][2].ptr));
				RNSSet(rns, RNS_F_RIGHT_VEL_PID, *(floatList[page][3].ptr), *(floatList[page][4].ptr), *(floatList[page][5].ptr));
				RNSSet(rns, RNS_B_LEFT_VEL_PID,  *(floatList[page][6].ptr), *(floatList[page][7].ptr), *(floatList[page][8].ptr));
				RNSSet(rns, RNS_B_RIGHT_VEL_PID, *(floatList[page][9].ptr), *(floatList[page][10].ptr), *(floatList[page][11].ptr));
			}
			tunePending();
			state = 0;
		}
		else{
			HAL_UART_Receive_DMA(tuneUart, &tuneHeader, 1);
			state = 1;
		}
		break;
	case 3:
		if(tuneBuffer[5] == 0x31 && tuneBuffer[6] == 0x40){
			*((int *)(intList[page][tuneBuffer[0]].ptr)) = *((int *)&tuneBuffer[1]);
		}

		if(tuneBuffer[5] == 0x20 && tuneBuffer[6] == 0x00){
			*(floatList[page][tuneBuffer[0]].ptr) = *((float *)&tuneBuffer[1]);
		}
		tuneBuffer[5] = tuneBuffer[6] = 0;
		HAL_UART_Receive_DMA(tuneUart, &tuneHeader, 1);
		state = 1;
		break;
	}
}

uint32_t tuneWriteInt(void){
	uint32_t success=0;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_6;
	EraseInitStruct.NbSectors     = 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return HAL_FLASH_GetError ();
	}
	for(int i=0; i<tunePage; i++){
		uint32_t Address = intAddress[i];
		for(int j=0; j<numInt[i]; j++){
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *((int *)(intList[i][j].ptr)))==HAL_OK){
				success ++;
			}
			Address += 4;
		}
	}

	HAL_FLASH_Lock();
	return success;
}

uint32_t tuneWriteFloat(void){

	uint32_t success=0;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_7;
	EraseInitStruct.NbSectors     = 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return HAL_FLASH_GetError ();
	}
	for(int i=0; i<tunePage; i++){
		uint32_t Address = floatAddress[i];
		for(int j=0; j<numFloat[i]; j++){
			uint32_t temp;
			memcpy(&temp, floatList[i][j].ptr, 4);
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, temp)==HAL_OK){
				success ++;
			}
			Address += 4;
		}
	}
	HAL_FLASH_Lock();
	return success;
}

void tuneRead(void){
	for(int page=0; page<tunePage; page++){
		uint32_t Address = intAddress[page];
		for(int i=0; i<numInt[page]; i++){
			memcpy(intList[page][i].ptr, (__IO uint32_t *)Address, 4);
			Address += 4;
		}
		Address = floatAddress[page];
		for(int i=0; i<numFloat[page]; i++){
			memcpy(floatList[page][i].ptr, (__IO uint32_t *)Address, 4);
			Address += 4;
		}
	}
}

void tunePID(UART_HandleTypeDef* huartx, PSxBT_t* ps4, RNS_interface_t* rns, float base_type){
	float target_speed = 3.0, base = base_type;
	int arr_x = 0, arr_y = 0, buf = 0;
	float PID[4][3]={{AP, AI, AD},
			{BP, BI, BD},
			{CP, CI, CD},
			{DP, DI, DD}};

	while(tune.PID){
		if(ps4->button == R1){		//adjust speed
			while(ps4->button == R1);
			target_speed += 1.0;
			if(target_speed > 6.0)
				target_speed = 6.0;
		}else if(ps4->button == L1){
			while(ps4->button == L1);
			target_speed -= 1.0;
			if(target_speed < 1.0)
				target_speed = 1.0;
		}

		LegacyMODN(ps4, rns);

		if(ps4->button == UP){		//direction
			uint32_t tick = HAL_GetTick();
			float a, b, c, d;
			while(ps4->button == UP){
				if(base == fwd_omni){
					RNSVelocity(target_speed, target_speed, target_speed, target_speed, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					d = rns->enq.enq_buffer[3].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f\n",
							target_speed, a, b, c, d);
					if(HAL_GetTick()-tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
				else if(base == tri_omni){
					RNSVelocity(target_speed, target_speed, 0.0, 0.0, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, 0.000\n",
							target_speed, a, b, c);
					if(HAL_GetTick()-tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
			}
			RNSStop(rns);
			memset(data, 0, 100);
		}else if(ps4->button == LEFT){
			uint32_t tick = HAL_GetTick();
			float a, b, c, d;
			while(ps4->button == LEFT){
				if(base == fwd_omni){
					RNSVelocity(-target_speed, target_speed, target_speed, -target_speed, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					d = rns->enq.enq_buffer[3].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f\n",
							target_speed, -a, b, c, -d);
					if(HAL_GetTick() - tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
				else if(base == tri_omni){
					RNSVelocity((0.5 * target_speed), (-0.5 * target_speed), -target_speed, 0.0, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, 0.000\n",
							target_speed, 2 * a, -2 * b, -c);
					if(HAL_GetTick()-tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
			}
			RNSStop(rns);
			memset(data, 0, 100);
		}else if(ps4->button == DOWN){
			uint32_t tick = HAL_GetTick();
			float a, b, c, d;
			while(ps4->button == DOWN){
				if(base == fwd_omni){
					RNSVelocity(-target_speed, -target_speed, -target_speed, -target_speed, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					d = rns->enq.enq_buffer[3].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f\n",
							target_speed, -a, -b, -c, -d);
					if(HAL_GetTick() - tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
				else if(base == tri_omni){
					RNSVelocity(-target_speed, -target_speed, 0.0, 0.0, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, 0.000\n",
							target_speed, -a, -b, c);
					if(HAL_GetTick()-tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
			}
			RNSStop(rns);
			memset(data, 0, 100);
		}else if(ps4->button == RIGHT){
			uint32_t tick = HAL_GetTick();
			float a, b, c, d;
			while(ps4->button == RIGHT){
				if(base == fwd_omni){
					RNSVelocity(target_speed, -target_speed, -target_speed, target_speed, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					d = rns->enq.enq_buffer[3].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f\n",
							target_speed, a, -b, -c, d);
					if(HAL_GetTick() - tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
				else if(base == tri_omni){
					RNSVelocity((-0.5 * target_speed), (0.5 * target_speed), target_speed, 0.0, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, 0.000\n",
							target_speed, -2 * a, 2 * b, c);
					if(HAL_GetTick()-tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
			}
			RNSStop(rns);
			memset(data, 0, 100);
		}

		if(ps4->button == CROSS){	//stop NAVI + plot graph
			while(ps4->button == CROSS);
			RNSStop(rns);
			uint8_t terminatePlot = 0x15;
			HAL_UART_Transmit(huartx, &terminatePlot, 1, 1);
		}

		if(ps4->button == TRIANGLE){	//TRI omni wheel
			while(ps4->button == TRIANGLE);
			base = tri_omni;
		}
		if(ps4->button == SQUARE){	//FOUR omni wheel
			while(ps4->button == SQUARE);
			base = fwd_omni;
		}
		if(ps4->button == SHARE){	//update PID
			while(ps4->button == SHARE);
			while(ps4->button != SHARE){
				if(ps4->button == SQUARE){	//print
					while(ps4->button == SQUARE);
					sprintf(data, "%.3f, %.3f, %.3f\n%.3f, %.3f, %.3f\n%.3f, %.3f, %.3f\n%.3f, %.3f, %.3f\n"
							, AP, AI, AD, BP, BI, BD, CP, CI, CD, DP, DI, DD);
					HAL_Delay(50);
					HAL_UART_Transmit(huartx, (uint8_t*)&data, strlen(data), 50);
				}
				if(ps4->button == UP){
					while(ps4->button == UP);
					arr_y ++;
					if(arr_y > 3)	arr_y = 3;
					sprintf(data, "arr y = %d\n", arr_y + 1);
					UARTPrintString(huartx, data);
				}
				if(ps4->button == DOWN){
					while(ps4->button == DOWN);
					arr_y --;
					if(arr_y < 0)	arr_y = 0;
					sprintf(data, "arr y = %d\n", arr_y + 1);
					UARTPrintString(huartx, data);
				}
				if(ps4->button == RIGHT){
					while(ps4->button == RIGHT);
					arr_x ++;
					if(arr_x > 2)	arr_x = 2;
					sprintf(data, "arr x = %d\n", arr_x + 1);
					UARTPrintString(huartx, data);
				}
				if(ps4->button == LEFT){
					while(ps4->button == LEFT);
					arr_x --;
					if(arr_x < 0)	arr_x = 0;
					sprintf(data, "arr x = %d\n", arr_x + 1);
					UARTPrintString(huartx, data);
				}
				if(ps4->button == L1){
					while(ps4->button == L1);
					buf --;
				}
				if(ps4->button == R1){
					while(ps4->button == R1);
					buf ++;
				}
				if(ps4->button == CIRCLE){
					while(ps4->button == CIRCLE);
					if(arr_x == 0)
						PID[arr_y][arr_x] = PID[arr_y][arr_x] + buf * 0.2;
					else if(arr_x == 1)
						PID[arr_y][arr_x] = PID[arr_y][arr_x] + buf * 0.2;
					else if(arr_x == 2)
						PID[arr_y][arr_x] = PID[arr_y][arr_x] + buf * 0.005;
					sprintf(data, "pid[%d][%d] = %.3f, buf = %d\n", arr_y + 1, arr_x + 1, PID[arr_y][arr_x], buf);
					HAL_Delay(50);
					HAL_UART_Transmit(huartx, (uint8_t*)&data, strlen(data), 50);
					AP = PID[0][0]; AI = PID[0][1]; AD = PID[0][3];
					BP = PID[1][0]; BI = PID[1][1]; BD = PID[1][3];
					CP = PID[2][0]; CI = PID[2][1]; CD = PID[2][3];
					DP = PID[3][0]; DI = PID[3][1]; DD = PID[3][3];
				}
				if(ps4->button == TRIANGLE){
					while(ps4->button == TRIANGLE);
					RNSSet(rns, RNS_F_LEFT_VEL_PID,  PID[0][0],  PID[0][1],  PID[0][2]);
					RNSSet(rns, RNS_F_RIGHT_VEL_PID,  PID[1][0],  PID[1][1],  PID[1][2]);
					RNSSet(rns, RNS_B_LEFT_VEL_PID,  PID[2][0],  PID[2][1],  PID[2][2]);
					RNSSet(rns, RNS_B_RIGHT_VEL_PID,  PID[3][0],  PID[3][1],  PID[3][2]);
					break;
				}
			}
		}
	}
}

void tunePIDex(UART_HandleTypeDef* huartx, PSxBT_t* ps4, RNS_interface_t* rns, float base_type){
	//USED TO TUNE PID OTHER THAN NAVI
	AP = 0.05; AI = 1.0; AD = 0.03;		//imu
	float target_speed = 3.0, target_angle = 0.0, buf = 0.0, base = base_type;
	int arr_x = 0, arr_y = 0;
	float PID[4][3]={{AP, AI, AD},
			{BP, BI, BD},
			{CP, CI, CD},
			{DP, DI, DD}};
	while(tune.PID){
		if(ps4->button == R1){		//adjust speed
			while(ps4->button == R1);
			target_angle += buf;
			if(target_angle >= 180)
				target_angle = 180;
			RNSIMURotate(target_angle, rns);
		}else if(ps4->button == L1){
			while(ps4->button == L1);
			target_angle -= buf;
			if(target_angle <= -180)
				target_angle = -180;
			RNSIMURotate(target_angle, rns);
		}

		LegacyMODN(ps4, rns);

		if(ps4->button == OPTION){
			while(ps4->button == OPTION);
			buf = 90.0;
		}
		if(ps4->button == UP){
			while(ps4->button == UP);
			buf += 2.0;
			sprintf(data, "d_arg = %.1f", buf);
			HAL_UART_Transmit(huartx, (uint8_t *)&data, strlen(data), 10);

		}else if(ps4->button == DOWN){
			while(ps4->button == DOWN);
			buf -= 2.0;
			sprintf(data, "d_arg = %.1f", buf);
			HAL_UART_Transmit(huartx, (uint8_t *)&data, strlen(data), 10);
		}else if(ps4->button == LEFT){
			uint32_t tick = HAL_GetTick();
			float a, b, c, d;
			while(ps4->button == LEFT){
				if(base == fwd_omni){
					RNSVelocity(-target_speed, target_speed, -target_speed, target_speed, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					d = rns->enq.enq_buffer[3].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f\n",
							target_speed, -a, b, -c, d);
					if(HAL_GetTick() - tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
				else if(base == tri_omni){
					RNSVelocity(-target_speed, target_speed, -target_speed, 0.0, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, 0.000\n",
							target_speed, -a, b, -c);
					if(HAL_GetTick()-tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
			}
			RNSStop(rns);
			memset(data, 0, 100);
		}else if(ps4->button == RIGHT){
			uint32_t tick = HAL_GetTick();
			float a, b, c, d;
			while(ps4->button == RIGHT){
				if(base == fwd_omni){
					RNSVelocity(target_speed, -target_speed, target_speed, -target_speed, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					d = rns->enq.enq_buffer[3].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f\n",
							target_speed, a, -b, c, -d);
					if(HAL_GetTick() - tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
				else if(base == tri_omni){
					RNSVelocity(target_speed, -target_speed, target_speed, 0.0, rns);
					RNSEnquire(RNS_VEL_BOTH, rns);
					a = rns->enq.enq_buffer[0].data;
					b = rns->enq.enq_buffer[1].data;
					c = rns->enq.enq_buffer[2].data;
					sprintf(data, "%.3f, %.3f, %.3f, %.3f, 0.000\n",
							target_speed, a, -b, c);
					if(HAL_GetTick()-tick >= 20){
						tick = HAL_GetTick();
						UARTPrintString(huartx, data);
					}
				}
			}
			RNSStop(rns);
			memset(data, 0, 100);
		}

		if(ps4->button == CROSS){	//stop NAVI + plot graph
			while(ps4->button == CROSS);
			RNSStop(rns);
			uint8_t terminatePlot = 0x15;
			HAL_UART_Transmit(huartx, &terminatePlot, 1, 1);
		}

		if(ps4->button == TRIANGLE){	//TRI omni wheel
			while(ps4->button == TRIANGLE);
			base = tri_omni;
		}
		if(ps4->button == SQUARE){	//FOUR omni wheel
			while(ps4->button == SQUARE);
			base = fwd_omni;
		}
		if(ps4->button == SHARE){	//update PID
			while(ps4->button == SHARE);
			while(ps4->button != SHARE){
				if(ps4->button == SQUARE){	//print
					while(ps4->button == SQUARE);
					sprintf(data, "%.3f, %.3f, %.3f\n%.3f, %.3f, %.3f\n%.3f, %.3f, %.3f\n%.3f, %.3f, %.3f\n"
							, AP, AI, AD, BP, BI, BD, CP, CI, CD, DP, DI, DD);
					HAL_Delay(50);
					HAL_UART_Transmit(huartx, (uint8_t*)&data, strlen(data), 50);
				}
				if(ps4->button == UP){
					while(ps4->button == UP);
					arr_y ++;
					if(arr_y > 3)	arr_y = 3;
					sprintf(data, "arr y = %d\n", arr_y + 1);
					UARTPrintString(huartx, data);
				}
				if(ps4->button == DOWN){
					while(ps4->button == DOWN);
					arr_y --;
					if(arr_y < 0)	arr_y = 0;
					sprintf(data, "arr y = %d\n", arr_y + 1);
					UARTPrintString(huartx, data);
				}
				if(ps4->button == RIGHT){
					while(ps4->button == RIGHT);
					arr_x ++;
					if(arr_x > 2)	arr_x = 2;
					sprintf(data, "arr x = %d\n", arr_x + 1);
					UARTPrintString(huartx, data);
				}
				if(ps4->button == LEFT){
					while(ps4->button == LEFT);
					arr_x --;
					if(arr_x < 0)	arr_x = 0;
					sprintf(data, "arr x = %d\n", arr_x + 1);
					UARTPrintString(huartx, data);
				}
				if(ps4->button == L1){
					while(ps4->button == L1);
					buf --;
				}
				if(ps4->button == R1){
					while(ps4->button == R1);
					buf ++;
				}
				if(ps4->button == CIRCLE){
					while(ps4->button == CIRCLE);
					if(arr_x == 0)
						PID[arr_y][arr_x] = PID[arr_y][arr_x] + buf * 0.1;
					else if(arr_x == 1)
						PID[arr_y][arr_x] = PID[arr_y][arr_x] + buf * 0.2;
					else if(arr_x == 2)
						PID[arr_y][arr_x] = PID[arr_y][arr_x] + buf * 0.005;
					sprintf(data, "pid[%d][%d] = %.3f, buf = %.0f\n", arr_y + 1, arr_x + 1, PID[arr_y][arr_x], buf);
					HAL_Delay(50);
					HAL_UART_Transmit(huartx, (uint8_t*)&data, strlen(data), 50);
					AP = PID[0][0]; AI = PID[0][1]; AD = PID[0][3];
					BP = PID[1][0]; BI = PID[1][1]; BD = PID[1][3];
					CP = PID[2][0]; CI = PID[2][1]; CD = PID[2][3];
					DP = PID[3][0]; DI = PID[3][1]; DD = PID[3][3];
				}
				if(ps4->button == TRIANGLE){
					while(ps4->button == TRIANGLE);
					RNSSet(rns, RNS_ROTATE_PID,  PID[0][0],  PID[0][1],  PID[0][2]);
//					RNSSet(rns, RNS_F_RIGHT_VEL_PID,  PID[1][0],  PID[1][1],  PID[1][2]);
//					RNSSet(rns, RNS_B_LEFT_VEL_PID,  PID[2][0],  PID[2][1],  PID[2][2]);
//					RNSSet(rns, RNS_B_RIGHT_VEL_PID,  PID[3][0],  PID[3][1],  PID[3][2]);
					break;
				}
			}
		}
	}
}

void tunePPlan(UART_HandleTypeDef* huartx, PSxBT_t* ps4, RNS_interface_t* rns){
	static int pppath = 0, MODNflag = 0;
	while(tune.PP){
		if(ps4->button == R1){		//choose path
			while(ps4->button == R1){}
			pppath ++;
			if(pppath > 8)
				pppath = 8;
		}else if(ps4->button == L1){
			while(ps4->button == L1){}
			pppath --;
			if(pppath < 0)
				pppath = 0;
		}

		if(ps4->button == OPTION)
		{
			ROS_Read_Flash();
			while(ps4->button == OPTION);
		}

//		if(ps4->button == SQUARE)
//		{
//			MODNflag = 0;
//			while(ps4->button == SQUARE);
//			if(test_point_num)
//				RNSPPstart_PS(test_points, test_point_num, rns);
//		}

		if(ros.ros_path_start){
			MODNflag = 0;
			ros.ros_path_start = 0;
			if(pppath == 1 || pppath == 3){
				RNSSet(rns, RNS_RESET_POS);
				RNSSet(rns, RNS_PP_Reset);
			}
			RNSPPstart_PS(PP_Points[path_index - 1], point_num[path_index - 1], rns);
		}
		else if(ros.ros_stop){
			MODNflag = 1;
			ros.ros_stop = 0;
			RNSStop(rns);
		}
		else if(ros.ros_test_start){
			MODNflag = 0;
			ros.ros_test_start = 0;
			RNSPPstart_PS(test_points, test_point_num, rns);
		}

		if(ps4->button == CIRCLE){		//run path plan
			MODNflag = 0;
			while(ps4->button == CIRCLE);
			switch (pppath){
			case 0:
				RNSStop(rns);
				break;
			case 1:
				RNSSet(rns, RNS_RESET_POS);
				RNSSet(rns, RNS_PP_Reset);
				RNSSet(rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
				registerPath(pathA, npA, point1, point2, point3);
				RNSPPstart(pathA, npA, rns);
				break;
			case 2:
				RNSSet(rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
				registerPath(pathB, npB, point5, point6, point7);
				RNSPPstart(pathB, npB, rns);
				break;
			case 3:
				RNSSet(rns, RNS_RESET_POS);
				RNSSet(rns, RNS_PP_Reset);
				RNSSet(rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
				registerPath(pathC, npC, point9, point10, point11);
				RNSPPstart(pathC, npC, rns);
				break;
			case 4:
				RNSSet(rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
				registerPath(pathD, npD, point12, point13, point14);
				RNSPPstart(pathD, npD, rns);
				break;
			default:
				pppath = 0;
				break;
			}
		}

		//LED indicator
		switch (pppath){
		case 0:
			GPIOD_OUT->bit8 = 0;
			GPIOD_OUT->bit9 = 0;
			GPIOA_OUT->bit7 = 0;
			GPIOA_OUT->bit6 = 0;
			GPIOA_OUT->bit5 = 0;
			GPIOA_OUT->bit4 = 0;
			break;
		case 1:
			GPIOD_OUT->bit9 = 1;
			GPIOA_OUT->bit8 = 0;
			break;
		case 2:
			GPIOA_OUT->bit8 = 1;
			GPIOA_OUT->bit7 = 0;
			break;
		case 3:
			GPIOA_OUT->bit7 = 1;
			GPIOA_OUT->bit6 = 0;
			break;
		case 4:
			GPIOA_OUT->bit6 = 1;
			GPIOA_OUT->bit5 = 0;
			break;
		case 5:
			GPIOA_OUT->bit5 = 1;
			GPIOA_OUT->bit4 = 0;
			break;
		case 6:
			GPIOA_OUT->bit5 = 0;
			GPIOA_OUT->bit4 = 1;
			break;
		case 7:
			GPIOA_OUT->bit5 = 1;
			GPIOA_OUT->bit4 = 1;
			break;
		case 8:
			GPIOA_OUT->bit5 = 0;
			GPIOA_OUT->bit4 = 0;
			GPIOA_OUT->bit7 = 1;
			GPIOA_OUT->bit6 = 1;
			break;
		}

		if(ps4->button == TRIANGLE && rns->RNS_data.common_instruction != RNS_BUSY){	//stickfance
			while(ps4->button == TRIANGLE);
			if(pppath == 1){
				while (lim_sw1 == 1|| lim_sw2 == 1 || lim_sw3 == 1){
					if(lim_sw1 == 1 && lim_sw2 == 1)
						RNSVelocity(1.75, 1.75, 0.0, 0.0, rns);
					else if(lim_sw2 == 1)
						RNSVelocity(1.50, 2.0, 0.0, 0.0, rns);
					else if(lim_sw1 == 1)
						RNSVelocity(2.0, 1.5, 0.0, 0.0, rns);
					else if(lim_sw3 == 1)
						RNSVelocity(2.0, 1.5, -1.50, 0.0, rns);
					if(ps4->button == CROSS){
						RNSStop(rns);
						break;
					}
				}
			}
			else if(pppath == 3){
				while (lim_sw2 == 1|| lim_sw3 == 1 || lim_sw4 == 1){
					if(lim_sw3 == 1 && lim_sw4 == 1)
						RNSVelocity(0.0, -1.75, 1.75, 0.0, rns);
					else if(lim_sw3 == 1)
						RNSVelocity(0.0, -2.0, 1.50, 0.0, rns);
					else if(lim_sw4 == 1)
						RNSVelocity(0.0, -1.5, 2.0, 0.0, rns);
					else if(lim_sw2 == 1)
						RNSVelocity(1.75, -1.25, 1.75, 0.0, rns);
					if(ps4->button == CROSS){
						RNSStop(rns);
						break;
					}
				}
			}
			RNSStop(rns);
			while(ps4->button == CROSS);
		}

		if(ps4->button == CROSS){	//stop + manual NAVI mode
			MODNflag = 1;
			RNSStop(rns);
			while(ps4->button == CROSS);
			if(ps4->button == L1_CROSS){
				RNSSet(rns, RNS_RESET_POS);
				RNSSet(rns, RNS_PP_Reset);
			}
		}

		if(MODNflag)
			realMODN(ps4, rns);
	}
}

