/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */
int pp_start = 0, state=0;

uint8_t buffer[9];
uint8_t rnsC=0;
int main(void)
{

	set();

	const osThreadAttr_t MainTask_attributes = {
			.name = "MainTask",
			.stack_size = 1024 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t EmergencyTask_attributes = {
			.name = "EmergencyTask",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t TuneTask_attributes = {
			.name = "TuneTask",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityAboveNormal,
	};

	const osThreadAttr_t LagoriTask_attributes = {
			.name = "LagoriTask",
			.stack_size = 1024 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t BLDCTask_attributes = {
			.name = "BLDCTask",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osSemaphoreAttr_t TuneSemaphore_attributes = {
			.name = "TuneSemaphore"
	};

//	ILI9341_Init();
////	ILI9341_Set_Rotation(1);
//	ILI9341_Fill_Screen(BLACK);
//	ILI9341_Init_List();

	sys.control = 1;
//	sys.tunePid = 1;
//	sys.tunePidex = 1;
//	sys.tunePP = 1;
//	sys.stickf = 1;

	osKernelInitialize();
	MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
	EmergencyTaskHandle = osThreadNew(EmergencyTask, NULL, &EmergencyTask_attributes);
	TuneTaskHandle = osThreadNew(TuneTask, NULL, &TuneTask_attributes);
	LagoriTaskHandle = osThreadNew(LagoriTask, NULL, &LagoriTask_attributes);
	BLDCTaskHandle = osThreadNew(BLDCTask, NULL, &BLDCTask_attributes);
	TuneSemaphore = osSemaphoreNew(1, 0, &TuneSemaphore_attributes);
//	led2 = 1;
	osKernelStart();

	while(1){
		if(!PB2){
			sprintf(data, "%.3f %.3f %.3f\n", tfmini.dist, tfmini.str, tfmini.temp);
			UARTSend
			Await(200);
		}
		if(ps4.button == SQUARE){
			while(ps4.button == SQUARE);
			registerPath(ToBallRackA, 3, point1, point2, point3);
			RNSPPstart(ToBallRackA, 3, &rns);
		}
	}
}

void TIM6_DAC_IRQHandler(void)
{
	led1 = !led1;
//	if(rns.RNS_data.common_instruction != RNS_WAITING){
//		if(rnsC >= 10){
//			CAN_Rns_TxSync(&hcan1);
//			rnsC = 0;
//		}
//		else
//			rnsC ++;
//	}
//	RNSSync(&rns);
//	UARTPrintString(&huart2, data);
	if(sys.manual && sys.control){
		//led indicator
		exled1 = 0;
		exled2 = 0;
		exled3 = 0;
		exled4 = 0;
		exled5 = 0;
		exled6 = 0;
		if(lagori.bldclayer >= 1)			exled1 = 1;
		if(lagori.bldclayer == 1)			exled2 = 1;
		else if(lagori.bldclayer == 2)		exled3 = 1;
		else if(lagori.bldclayer == 3)		exled4 = 1;
		else if(lagori.bldclayer == 4)		exled5 = 1;
		else if(lagori.bldclayer == 5)		exled6 = 1;

		else if(lagori.bldcflayer == 1)		exled2 = 1;
		else if(lagori.bldcflayer == 2)		exled3 = 1;
		else if(lagori.bldcflayer == 3)		exled4 = 1;
		else if(lagori.bldcflayer == 4)		exled5 = 1;
		else if(lagori.bldcflayer == 5)		exled6 = 1;
	}
	HAL_TIM_IRQHandler(&htim6);
}

void MainTask(void *argument)
{
	led3 = 1;
	fx.flags = 0;
	while(1)
	{
		while(sys.control){
			if(ps4.button == SHARE){
				sys.manual = 1;
				seeker_mode();
			}
			if(ps4.button == OPTION){
				led3 = 0;
				sys.manual = 0;
				hitter_mode();
			}

			if(!PB1){
//				while(!PB1);
//				svBallvp(2550)
//				svBallhp(2050)
//				svLGF1(875)
//				svLGF2(1775)
				svBRA
				svBRdown
				svLGF1(1275)
				svLGF2(1600)
			}
			if(!PB2){
				while(!PB2);
				ROS_Read_Flash();
			}

			button();
			limit_sw();
//			analog_sr();

			if(ps4.button == L3){
				press = 0;
				sprintf(data, "Press SQUARE to start rns enq\n");
				UARTSend
				while(ps4.button == L3);
				while(ps4.button != L3){
					functest();
					enq();
					if(ps4.button == SQUARE){
						counter = 0;
						while(ps4.button == SQUARE);
						if(counter > 2000)
							n = 6;
						else
							n++;
					}
				}
				while(ps4.button == L3);
			}

			if(ps4.button == R3){
				while(ps4.button == R3);
				while(ps4.button != R3){
					KFilterAdjust();
				}
				while(ps4.button == R3);
			}

			if(ps4.button == L1 && fabs(ps4.joyR_x) >= 0.5)
				LGPW(ps4.joyR_x * 9000)
			if(fabs(ps4.joyR_y) >= 0.5)
				LFPW(ps4.joyR_y * 9000)
			else{
				LFPW(0)
				LGPW(0)
			}

			if(ps4.button == R1){
				setSpeedMODN(7.0);
			}else{
				setSpeedMODN(4.0);
			}

			realMODN(&ps4, &rns);

			if(ps4.button == L1){
				if(pp.cyon == 0)
					BgripON
				else
					BgripOFF
				pp.cyon = !pp.cyon;
				while(ps4.button == L1);
			}

			//		//diagnostic
			//		ILI9341_Update_List();
		}

		while(sys.tunePid){
			tune.PID = 1;
			tunePID(tuneUart, &ps4, &rns, (float) fwd_omni);
		}

		while(sys.tunePidex){
			tune.PIDex = 1;
			tunePIDex(tuneUart, &ps4, &rns, (float) fwd_omni);
		}

		while(sys.tunePP){
			tune.PP = 1;
			RNSSet(&rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
			tunePPlan(tuneUart, &ps4, &rns);
		}

		while(sys.stickf){
			if(!sys.ppend)
				realMODN(&ps4, &rns);
			if(pp.path == 0)
				exled1 = 0;
			if(pp.path == 1){
				exled1 = 1;
				exled2 = 0;
			}
			if(pp.path == 2){
				exled1 = 0;
				exled2 = 1;
				exled3 = 0;
			}
			if(pp.path == 3){
				exled2 = 0;
				exled3 = 1;
				exled4 = 0;
			}
			if(pp.path == 4){
				exled3 = 0;
				exled4 = 1;
				exled5 = 0;
			}
			if(pp.path == 5){
				exled4 = 0;
				exled5 = 1;
				exled6 = 0;
			}
			if(pp.path == 6){
				exled5 = 0;
				exled6 = 1;
			}

			if(ps4.button == L1){
				while(ps4.button == L1);
				pp.path --;
				if(pp.path < 0)	pp.path = 0;
			}
			if(ps4.button == R1){
				while(ps4.button == R1);
				pp.path ++;
				if(pp.path > 6)	pp.path = 6;
			}
			if(ps4.button == CROSS){
				sys.ppend = 0;
				sys.control = 0;
			}
			if(ps4.button == TRIANGLE){
				while(ps4.button == TRIANGLE);
				sys.ppend = 1;
				pp.base = 1;
				sys.auto2manual = 0;
			}
			if(sys.ppend){
				if(pp.path == 1){
					while (lsl1|| lsl2 || lsr1){
						if((!(lsl2 || lsr1)) && lsl1)
							RNSVelocity(0.00, 1.50, 0.00, 1.00, &rns);
						else if(lsl1 || lsl2)
							RNSVelocity(1.25, 1.25, 1.25, 1.25, &rns);
						else if(lsr1)
							RNSVelocity(-0.75, 1.00, 1.00, -0.75, &rns);
						else
							RNSStop(&rns);
						if(ps4.button == CROSS){
							RNSStop(&rns);
							break;
						}
						Await(100);
					}
					RNSStop(&rns);
				}
				else if(pp.path == 2 || pp.path == 4){
						pp.laser = 1;
						if(pp.path == 2)
							z_tar = 90.0;
						else
							z_tar = 0.0;
					while(ps4.button != CROSS){
						sys.control = 1;
					}
				}
				else if(pp.path == 3){
					//stick france
					while (lsl2|| lsr1 || lsr2){
						if((!(lsl2 || lsr1)) && lsr2)
							RNSVelocity(0.00, 0.00, 1.50, -1.00, &rns);
						else if(lsr1 || lsr2)
							RNSVelocity(-0.75, 0.75, 2.00, -1.50, &rns);
						else if(lsl2)
							RNSVelocity(0.75, 1.00, 1.00, 0.75, &rns);
						else{
							RNSStop(&rns);
						}
						if(ps4.button == CROSS){
							RNSStop(&rns);
							break;
						}
						Await(100);
					}
					RNSStop(&rns);
				}
				else if(pp.path == 5){	//lagori
					pp.base = 1;
					while(pp.base){
						RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
						zrad = rns.enq.enq_buffer[3].data;
						vz = 5.0 * zrad;
						if(vz < 0.5)		vz = 0.5;
						else if (vz > -0.5)	vz = -0.5;

						if(fabs(zrad) > z_err)
							RNSVelocity(vz, -vz, vz, -vz, &rns);
						else{
							if(tfmini.dist_err > 0.05)			RNSVelocity(-2.5, -2.5, -2.5, -2.5, &rns);
							else if(tfmini.dist_err < -0.25)	RNSVelocity(0.75, 0.75, 0.75, 0.75, &rns);
							else if(!(LGbasel))					RNSVelocity(1.00, -1.00, -1.00, 1.00, &rns);
							else if(!(LGbaser))					RNSVelocity(-1.00, 1.00, 1.00, -1.00, &rns);
							else{
								Await(150);
								RNSStop(&rns);
								pp.base = 0;
								pp.baseok = 1;
							}
						}
						if(ps4.button == CROSS){
							RNSStop(&rns);
							break;
						}
					}
					pp.base = 0;
				}
				else if(pp.path == 6){	//lagori (test without imu adjust
					pp.base = 1;
					while(pp.base){
						RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
						zrad = rns.enq.enq_buffer[3].data;
						vz = 5.0 * (zrad - 1.5708);
						if(vz < 0.5)		vz = 0.5;
						else if (vz > -0.5)	vz = -0.5;

						if(fabs(zrad - 1.5708) > z_err)
							RNSVelocity(vz, -vz, vz, -vz, &rns);
						else{
							if(tfmini.dist_err > 0.05)			RNSVelocity(-2.5, -2.5, -2.5, -2.5, &rns);
							else if(tfmini.dist_err < -0.25)	RNSVelocity(0.75, 0.75, 0.75, 0.75, &rns);
							else if(!(LGbasel))					RNSVelocity(1.00, -1.00, -1.00, 1.00, &rns);
							else if(!(LGbaser))					RNSVelocity(-1.00, 1.00, 1.00, -1.00, &rns);
							else{
								Await(150);
								RNSStop(&rns);
								pp.base = 0;
								pp.baseok = 1;
							}
						}
						if(ps4.button == CROSS){
							RNSStop(&rns);
							break;
						}
					}
					pp.base = 0;
				}
				sys.ppend = 0;
			}
		}
	}
}

void LagoriTask(void *argument){
	while(1){
		if(sys.manual)
			lagori_func();
	}
}

void BLDCTask(void *argument){
	while(1){
		if(sys.manual)
			lifting_func();
	}
}

void EmergencyTask(void *argument){
	while(1){
		enqpwLG = QEIRead(QEI1);
		enqpwLlf = QEIRead(QEI4);
		tfmini.dist_err = tfmini.dist - tfmini.dist_target;

		if(ps4.button == TOUCH){
			RNSStop(&rns);
			svCloseFlip
			svBRrealup
			LGPW(0);
			LFPW(0);
			sys.flags = 0;
			tune.flags = 0;
			pp.flags = 0;
			lagori.flags = 0;
//			VESCStop(&vesc1);
//			osThreadSuspend(FlywheelPitchPIDTaskHandle);
//			osThreadSuspend(FlywheelYawPIDTaskHandle);
			osThreadTerminate(MainTaskHandle);
			osThreadTerminate(BLDCTaskHandle);
			osThreadTerminate(LagoriTaskHandle);

			osDelay(5);

			const osThreadAttr_t MainTask_attributes = {
					.name = "MainTask",
					.stack_size = 256 * 4,
					.priority = (osPriority_t) osPriorityNormal,
			};

			const osThreadAttr_t BLDCTask_attributes = {
					.name = "BLDCTask",
					.stack_size = 256 * 4,
					.priority = (osPriority_t) osPriorityNormal,
			};

			const osThreadAttr_t LagoriTask_attributes = {
					.name = "LagoriTask",
					.stack_size = 1024 * 4,
					.priority = (osPriority_t) osPriorityNormal,
			};

			sys.control = 1;
			sys.auto2manual = 1;
			MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
			BLDCTaskHandle = osThreadNew(BLDCTask, NULL, &BLDCTask_attributes);
			LagoriTaskHandle = osThreadNew(LagoriTask, NULL, &LagoriTask_attributes);
		}
	}
}

void TuneTask(void *argument){
	tunePending();
	int page = -1;
	while(1){
		osSemaphoreAcquire(TuneSemaphore,osWaitForever);

		if(tuneCommand == '0'){
			page = 0;
			tuneInit(page);
		}
		if(tuneCommand == '1'){
			page = 1;
			tuneInit(page);
		}
		if(tuneCommand == '2'){
			page = 2;
			tuneInit(page);
		}
		if(tuneCommand == '3'){
			page = 3;
			tuneInit(page);
		}
		if(tuneCommand == '4'){
			page = 4;
			tuneInit(page);
		}

		if(tuneCommand == 'r'){
			tuneUpdate(page);
		}

		if(tuneCommand == 'w'){
			led3 = 0;
			led2 = 0;
			uint32_t writeSuccess=0;
			writeSuccess += tuneWriteInt();
			writeSuccess += tuneWriteFloat();

			if( writeSuccess == tuneWriteSuccess){
				HAL_UART_Transmit(tuneUart, (uint8_t *)((char *)"EEPROM WRITTEN\n"), 15, 10);
				led2 = 1;
			}else{
				HAL_UART_Transmit(tuneUart, (uint8_t *)((char *)"EEPROM WRITE FAIL\n"), 18, 10);
				led3 = 1;
			}
			tunePending();
		}

		if(tuneCommand == 'e'){
			tuneEdit(page, &rns);
		}

		if(tuneCommand == 'c'){
			sys.flags = 0;
			osDelay(10);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == tuneUart){
		osSemaphoreRelease(TuneSemaphore);
	}
	if(huart == tfmini.huartx){
		ledtick ++;
		if(ledtick >= 10){
			led2 = !led2;
			ledtick = 0;
		}
		TFmini_Handler(&tfmini);
	}
	else if(huart == ROS_UART){
		led3 = !led3;
		ROS_Handler();
	}else if(huart == esp32.huart){
		esp32Handler();
	}else if(huart == ps4.huartps4){
		PSx_HandlerUART(&ps4);
	}

}

/**
 * @brief  This function is executed in case of errorYaw occurrence.
 */
void Error_Handler(void)
{


}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
