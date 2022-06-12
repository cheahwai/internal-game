
/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"

float expath[1][7];
void set(void) {

	Initialize();
	PSxInitDMA(&ps4,&hi2c2);
	//	PSxInitUART(&ps4, &huart3);
	TIMxInit(&htim6, 20000, 84);
	RNS_config(&hcan1);
	led3 = 1;

	MODNInit(MODN_FWD_OMNI, 4.5, 0.625, 3.0, 0.03, 800);

	TFmini_Init(&tfmini, &huart4);

	tuneRegister(3, &huart2);
	tuneRead();
	setPP_points();
//	ROS_Init(&huart2);
//	ROS_Read_Flash();

	tfmini.dist_target = 0.45;

	svBRup
	svBRAback
	Await(1000);
	led3 = 0;
	svBRrealup
	BgripOFF
	svCloseFlip
	QEIReset(QEI1);
	QEIReset(QEI4);
	QEISwap(QEI1, QEI_Swap);
	QEISwap(QEI4, QEI_Swap);
}

void RNS_config(CAN_HandleTypeDef* hcanx) {

//	AP = 1.617; AI = 4.173; AD = 0.014;
//	BP = 1.62; BI = 4.249; BD = 0.014;
//	CP = 1.63; CI = 4.012; CD = 0.014;
//	DP = 1.66; DI = 4.264; DD = 0.014;

	AP = 1.46; AI = 2.722; AD = 0.010;
	BP = 1.50; BI = 2.696; BD = 0.010;
	CP = 1.49; CI = 2.549; CD = 0.010;
	DP = 1.53; DI = 2.748; DD = 0.010;

	RNSInit(hcanx, &rns);
	//Encoder dcba(0-swap, 1-keep)  BDC dir dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b00000101, (float) fwd_omni, (float) roboconPID);

	/*****************INITIALIZE ENC VALUE | ENC RATIO*****************/
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.05 / 4000.0 * 3.142, 2.0,
			0.05 / 4000.0 * 3.142, 2.0); //1.0 for nonswap , 2.0 for swap
//	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.051 / 4000.0 * 3.142, 2.0,
//			0.051 / 4000.0 * 3.142, 2.0); //1.0 for nonswap , 2.0 for swap
	RNSSet(&rns, RNS_F_KCD_PTD, 2593.721/2588.222, 0.125 * 31.42 / 2593.721);
	RNSSet(&rns, RNS_B_KCD_PTD, 2579.189/2572.120, 0.125 * 31.42 / 2579.189);

	/*****************INITIALIZE MOTOR MAX SPEED*****************/
	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 8.596, 19999.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 9.548, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 8.385, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 8.993, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  AP, AI, AD);
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, BP, BI , BD);
	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  CP, CI, CD);
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID,  DP, DI, DD);

	RNSSet(&rns, RNS_ROTATE_SATEU, 1.0, 1.0 / 180, 19999.0);
	RNSSet(&rns, RNS_ROTATE_PID,  0.05, 1.0, 0.03);

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 1.0, 0.5, 0.5);
	RNSSet(&rns, RNS_PPEndPID, 0.5, 0.1, 1.0);
	RNSSet(&rns, RNS_PPZPID, 1.0, 0.05, 0.2, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0);         // Change No. of Points in the Curved Path
}

void seeker_mode(void){
	while(sys.manual){
		//NAVI, servo, pw(h)
		if(pp.base > 0 && pp.base < 6){
			if(pp.base == 1){	//right
				target_enq = 10000;
				lagori.bldcstart = 1;
				RNSSet(&rns, RNS_RESET_POS);
				RNSSet(&rns, RNS_PP_Reset);
				registerPath(LBaseS, 1, pLSE);
				RNSSet(&rns, RNS_PPSetXYZerror, 0.05, 2.0, 0.025, 1.0);
				RNSPPstart(LBaseS, 1, &rns);
				Await(333);
				pLNE[0] = 7.2;
				pLNE[1] = 1.25;
				pLNE[2] = 5.20;
				pLNW[0] = 4.8;
				pLNW[1] = -0.75;
				pLNW[2] = 5.85;
				registerPath(LBaseN, 2, pLNE, pLNW);
				RNSSet(&rns, RNS_PPSetXYZerror, 0.20, 2.0, 0.10, 1.0);
				RNSPPstart(LBaseN, 2, &rns);
			}
			else if(pp.base == 2){
				target_enq = 10000;
				lagori.bldcstart = 1;
				RNSSet(&rns, RNS_RESET_POS);
				RNSSet(&rns, RNS_PP_Reset);
				RNSSet(&rns, RNS_PPSetXYZerror, 0.05, 2.0, 0.025, 1.0);
				registerPath(LBaseS, 1, pLSW);
				RNSPPstart(LBaseS, 1, &rns);
				Await(333);
				pLNW[0] = 7.2;
				pLNW[1] = -1.25;
				pLNW[2] = 5.35;
				pLNE[0] = 4.8;
				pLNE[1] = 0.75;
				pLNE[2] = 6.00;
				registerPath(LBaseN, 2, pLNW, pLNE);
				RNSSet(&rns, RNS_PPSetXYZerror, 0.20, 2.0, 0.10, 1.0);
				RNSPPstart(LBaseN, 2, &rns);
			}
			pp.base = 6;
		}
		else if(rns.RNS_data.common_instruction != RNS_BUSY){	// adjustment
			if(pp.base == 7){
				for(int i = 0; i < 2; i++){
					while(pp.base != 0){
						RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
						z = rns.enq.enq_buffer[2].data;
//				if(fabs(z - z_tar) > z_err)
//					RNSIMURotate(z_tar, &rns);
						if(z > 350)			z -= 360;
						else if(z < -10)	z += 360;
						vz = (z - z_tar) / 15.0;
						if(vz < 0.50)			vz = 0.50;
						else if(vz > 3.00)		vz = 3.00;
						else if (vz > -0.50)	vz = -0.50;
						else if (vz < -3.00)	vz = -3.00;

						if(fabs(z - z_tar) > z_err)
							RNSVelocity(vz, -vz, vz, -vz, &rns);
						else{
							if(tfmini.dist_err > 0.05)			RNSVelocity(-2.5, -2.5, -2.5, -2.5, &rns);
							else if(tfmini.dist_err < -0.30)	RNSVelocity(0.75, 0.75, 0.75, 0.75, &rns);
							else if(!(LGbasel))					RNSVelocity(1.00, -1.00, -1.00, 1.00, &rns);
							else if(!(LGbaser))					RNSVelocity(-1.00, 1.00, 1.00, -1.00, &rns);
							else{
								RNSStop(&rns);
								pp.base = 0;
							}
						}
						if(ps4.button == L1){
							RNSStop(&rns);
							break;
						}
					}
					RNSStop(&rns);
					Await(100);
					pp.base = 7;
				}
				pp.base = 0;
//				pp.baseok = 1;
			}
			else if(pp.baseleft)
				RNSVelocity(5.2, 5.2, 5.2, 5.2, &rns);
			else
				realMODN(&ps4, &rns);
		}

		if(ps4.button == R1)
			setSpeedMODN(7.6);
		else
			setSpeedMODN(4.5);

		//flipservo
		if(lagori.flip == 1){
			svFlip
		}else if(lagori.flip == 0){
			svNoFlip
		}

		//lagori gripper
		if(lagori.pwstate == 0){	//open
			lagori.gripok = 0;
			if(!LGopen)
				LGPW(12000)
			else{
				LGPW(0)
				lagori.pwstate = 3;
			}
		}
		else if(lagori.pwstate == 1){	//grip
			if(lsLGl && lsLGr){
				LGPW(-12000)
				if(LGclose || enqpwLG < dangerclose){
					LGPW(0)
					lagori.pwstate = 3;
					lagori.bldclayer = 5;
					lagori.flip = 2;
					svLGF1(1275)
					svLGF2(1525)
					lagori.bldclapp = 1;
				}
				if(LGclose){
					LGPW(0)
					lagori.pwstate = 3;
				}
			}else{
				Await(75);
				LGPW(0)
				lagori.gripok = 1;
				lagori.pwstate = 3;
				sprintf(data, "pwenq, %d\n", enqpwLG);
				UARTSend
			}
		}
		else if(lagori.pwstate == 2){	//close OR retry
			if(!(LGclose))	LGPW(-12000)
			else{
				LGPW(0)
				QEIReset(QEI1);
				lagori.pwstate = 3;
			}
		}
	}
}

void lagori_func (void){

	lagori.flip = 2;
	QEIReset(QEI4);
	svBRup
	svBRAback
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.051 / 4000.0 * 3.142, 2.0,
			0.051 / 4000.0 * 3.142, 2.0); //1.0 for nonswap , 2.0 for swap
	Await(500);
	BgripOFF
	svBRrealup
	svCloseFlip
	lagori.pwstate = 0;

	while(sys.manual){
		if(ps4.button == CROSS){		//close lagori gripper
				lagori.pwstate = 1;
			while(ps4.button == CROSS);
		}

		if(ps4.button == CIRCLE){		//flip + push
			while(ps4.button == CIRCLE);
			if(ps4.button == R1_CIRCLE){
				fx.svapp = 1;
				while(ps4.button == R1_CIRCLE);
				lagori.pwstate = 2;
//				lagori.bldcflayer = 2;
//				lagori.bldcfapp = 1;
				target_enq = svfms_layer - 500;
				lagori.bldcstart = 1;
				lagori.flip = 2;
				svLGF1(1150)
				svLGF2(1650)
			}
			else{
				lagori.pwstate = 0;
				lagori.bldclapp = 0;
				lagori.bldcfapp = 1;
//				Await(750);
//				while(lagori.bldcstart != 0){}
				while(enqpwLlf <= svfs_layer - 300);
				lagori.flip = 1;
				while(enqpwLlf <= target_enq);
				lagori.pwstate = 1;
				while(!lagori.gripok){
					if(LGclose || enqpwLG < dangerclose){
						lagori.pwstate = 3;
						LGPW(0)
						Await(100);
						lagori.bldclayer = 5;
						lagori.flip = 2;
						svLGF1(1150)
						svLGF2(1650)
						lagori.bldclapp = 1;
						break;
					}
					if(ps4.button == CIRCLE)
						lagori.pwstate = 1;
					if(ps4.button == L1)	//cancel
						break;
				}
				Await(100);
				if(ps4.button != L1){
					lagori.flip = 2;
					svLGF1(1250)
					svLGF2(1550)
				}
			}
		}

		if(ps4.button == TRIANGLE){		//servo
			while(ps4.button == TRIANGLE);
			if(!fx.svapp){
				lagori.flip = 2;
				svLGF1(1350)
				svLGF2(1450)
			}
			else{
				lagori.flip = 0;
			}
			fx.svapp = !fx.svapp;
		}

		if(ps4.button == SQUARE || pp.baseok){	//release lagori
			lagori.pwstate = 0;
			lagori.flip = 2;
			while(ps4.button == SQUARE);
			if(ps4.button == R1_SQUARE){	//retry state
				while(ps4.button == R1_SQUARE);
				lagori.pwstate = 2;
				lagori.flip = 2;
				svCloseFlip
				lagori.bldclayer = 0;
				lagori.bldclapp = 1;
			}
			else if(ps4.button == L1_SQUARE){	//ready state
				lagori.flip = 0;
				lagori.bldclayer = 0;
				lagori.bldclapp = 1;
				while(ps4.button == L1_SQUARE || ps4.button == SQUARE || ps4.button == L1);
			}
			else{				//wait to release lagori and give time to go back
				Await(350);
				pp.baseok = 0;
				pp.baseleft = 1;
				Await(375);
				pp.baseleft = 0;
				lagori.flip = 0;
				lagori.bldclayer = 0;
				lagori.bldclapp = 1;
			}
		}

		//auto pilling
		if(ps4.button == UP){
			while(ps4.button == UP);
			if(ps4.button == R1_UP){
				while(ps4.button == R1_UP);
				pp.base = 1;
				while(ps4.button == R1 || ps4.button == UP);
			}
			else if(ps4.button == L1_UP){
				while(ps4.button == L1_UP);
				pp.base = 2;
				while(ps4.button == L1 || ps4.button == UP);
			}
			else{
				pp.base = 7;
				z_tar = 0;
			}
		}
		if(ps4.button == DOWN){
			while(ps4.button == DOWN);
			pp.base = 7;
			z_tar = 180;
		}
		if(ps4.button == LEFT){
			while(ps4.button == LEFT);
			pp.base = 7;
			z_tar = 270;
		}
		if(ps4.button == RIGHT){
			while(ps4.button == RIGHT);
			pp.base = 7;
			z_tar = 90;
		}
		if(ps4.button == OPTION){
			while(ps4.button == OPTION);
			RNSSet(&rns, RNS_RESET_POS);
			RNSSet(&rns, RNS_PP_Reset);
			pLNE[0] = 1.5;
			pLNE[1] = 0.0;
			pLNE[2] = 0.0;
			for(int i = 0; i <= 6; i++){
				target_enq = 10000;
				lagori.bldcstart = 1;
//				if(i == 0)		pLNE[3] = 0.0;
//				else if(i == 1)	pLNE[3] = 45.0;
//				else if(i == 2)	pLNE[3] = -45.0;
//				else if(i == 3)	pLNE[3] = 0.0;
//				registerPath(LBaseN, 1, pLNE);
//				RNSSet(&rns, RNS_PPSetXYZerror, 0.10, 2.0, 0.05, 1.0);
//				RNSPPstart(LBaseN, 1, &rns);
//				while(rns.RNS_data.common_instruction == RNS_BUSY){
					if(!fx.svapp){
						lagori.flip = 2;
						svLGF1(1350)
						svLGF2(svlnf2);
					}
					else{
						lagori.flip = 2;
						svLGF1(svlnf);
						svLGF2(1450)
					}
					fx.svapp = !fx.svapp;
//					if(ps4.button == L1){
//						RNSStop(&rns);
//						break;
//					}
					Await(330);
//				}
				if(ps4.button == L1){
					RNSStop(&rns);
					break;
				}
			}
		}

		//manual pulley + no auto pilling
		if(ps4.button == L1){
			if(pp.base != 0){
				pp.base = 0;
				RNSStop(&rns);
			}
			lagori.bldcstart  = 0;
			if(fabs(ps4.joyR_y) >= 0.5)
				LFPW(ps4.joyR_y * 19999)
			else
				LFPW(0)
		}
		if(lagoricounter > 1000 && ps4.joyR_x >= 0.5){
			lagori.bldclayer ++;
			if(lagori.bldclayer > 5)
				lagori.bldclayer = 5;
			lagori.bldclapp = 1;
			lagoricounter = 0;
		}
		else if(lagoricounter > 1000 && ps4.joyR_x <= -0.5){
			lagori.bldclayer --;
			if(lagori.bldclayer < 0)
				lagori.bldclayer = 0;
			lagori.bldclapp = 1;
			lagoricounter = 0;
		}
	}
}

void lifting_func(void){
	int enqerr = 0;
	float pwspeed = 0;
	laserdir = 0;
	while(sys.manual){
		//auto pulley
		if(lagori.gripok){
			if(enqpwLG < LGsize2){
				lagori.bldclayer = 5;
				lagori.flip = 2;
				fx.svapp = 0;
				svLGF1(1275)
				svLGF2(1525)
			}
			else if(enqpwLG < LGsize3)	lagori.bldclayer = 4;
			else if(enqpwLG < LGsize4)	lagori.bldclayer = 3;
			else if(enqpwLG < LGsize5)	lagori.bldclayer = 2;
			else 						lagori.bldclayer = 1;
			//				if(lagori.bldcflayer < 5)	lagori.bldclayer --;
			lagori.bldclapp = 1;
			Await(5);
			lagori.gripok = 0;
		}

		// flip lagori
		if(LGHl)
			lagori.bldcflayer = 1;
		else if(LGHml)
			lagori.bldcflayer = 2;
		else if(LGHm)
			lagori.bldcflayer = 3;
		else if(LGHms)
			lagori.bldcflayer = 4;
		else
			lagori.bldcflayer = 5;

		if(lagori.bldclapp){
			if(lagori.bldclayer == 0)			target_enq = layerG;
			else if(lagori.bldclayer == 1)		target_enq = layer1;
			else if(lagori.bldclayer == 2)		target_enq = layer2;
			else if(lagori.bldclayer == 3)		target_enq = layer3;
			else if(lagori.bldclayer == 4)		target_enq = layer4;
			else if(lagori.bldclayer == 5)		target_enq = layer5;
			lagori.bldclapp = 0;
			lagori.bldcstart = 1;
		}
		else if(lagori.bldcfapp){
			if(lagori.bldcflayer == 1)			target_enq = svfl_layer;
			else if(lagori.bldcflayer == 2)		target_enq = svfml_layer;
			else if(lagori.bldcflayer == 3)		target_enq = svfm_layer;
			else if(lagori.bldcflayer == 4)		target_enq = svfms_layer;
			else if(lagori.bldcflayer == 5)		target_enq = svfs_layer;
			lagori.bldcfapp = 0;
			lagori.bldcstart = 1;
		}

		//if +ve is up, arr = 20000
		enqerr = target_enq - enqpwLlf;
		pwspeed = enqerr;
		if(enqerr >= 19999)						pwspeed = 19999;
		else if(enqerr <= 17500 && enqerr > 0)	pwspeed = 17500;
		else if(enqerr <= -19999)				pwspeed = -19999;
		else if(enqerr >= -17500 && enqerr < 0)	pwspeed = -17500;

		if(lagori.bldcstart){
			if(enqpwLlf >= 22500){
				LFPW(0);
				lagori.bldcstart = 0;
			}
			else if(target_enq == layer5 && enqerr <= 750){
				Await(150);
				LFPW(0);
				lagori.bldcstart = 0;
			}
			else if(target_enq == layerG && enqerr >= -750){
				Await(200);
				LFPW(0);
				lagori.bldcstart = 0;
			}
			else if(enqerr <= 750 && enqerr >= -750){	//stop
				Await(150);
				LFPW(0);
				lagori.bldcstart = 0;
			}
			else
				LFPW(pwspeed);
		}

//		else if(pp.laser && sys.manual == 0 && sys.auto2manual == 0){
//			RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
//			z = rns.enq.enq_buffer[2].data;
//			vz = (z - z_tar) / 15.0;
//			if(vz < 0.50)			vz = 0.50;
//			else if(vz > 3.00)		vz = 3.00;
//			else if (vz > -0.50)	vz = -0.50;
//			else if (vz < -3.00)	vz = -3.00;
//
//			if(pp.path == 2){
//				y_err = fAdist - laser_dist_y;
//				x_err = R1dist - laser_dist_x;
////				if(y_err > 1.5)		 vy = 4.0;
////				else if(y_err < 0.2) vy = 1.0;
////				else				 vy = 2.5;
//
//				if(rns.RNS_data.common_instruction != RNS_BUSY){
//					if(fabs(z - z_tar) > z_err)
//						RNSVelocity(vz, -vz, vz, -vz, &rns);
////					if(fabs(z - z_tar) > z_err)
////						RNSIMURotate(z_tar, &rns);
//					else{
////						if(fabs(y_err) >= 0.05){
////use signbit() to get the sign of err then give the direction
////							laserdir = (signbit(y_err) == 0)? 1 : -1;
////							v1 *= laserdir;
////							v2 *= -laserdir;
////							v2 *= -laserdir;
////							v4 *= laserdir;
////							RNSVelocity(v1, v2, v3, v4, &rns);
////						}
//						if(y_err >= 0.03)
//							RNSVelocity(-0.5, -0.5, -0.5, -0.5, &rns);
//						else if(y_err <= -0.03)
//							RNSVelocity(0.5, 0.5, 0.5, 0.5, &rns);
////						else if(x_err > 0.10){
////							RNSVelocity(-0.75, 0.75, 0.75, -0.75, &rns);
////						}
//						else{
//							RNSStop(&rns);
//							pp.laser = 0;
//							sys.auto2manual = 1;
//							pp.cyon = 0;
//						}
//					}
//					if(ps4.button == CROSS){
//						RNSStop(&rns);
//						pp.cyon = 0;
//						sys.auto2manual = 1;
//						pp.laser = 0;
//					}
//				}
//			}
//
//			if(pp.path == 4){
//				y_err = R1dist - laser_dist_y;
//				x_err = fBdist - laser_dist_x;
//
//				if(rns.RNS_data.common_instruction != RNS_BUSY){
//
//					if(fabs(z - z_tar) > z_err)
//						RNSVelocity(vz, -vz, vz, -vz, &rns);
//					else{
//						if(x_err >= 0.03)
//							RNSVelocity(0.5, -0.5, -0.5, 0.5, &rns);
//						else if(x_err <= -0.03)
//							RNSVelocity(-0.5, 0.5, 0.5, -0.5, &rns);
////						else if(y_err > 0.10){
////							RNSVelocity(0.75, 0.75, 0.75, 0.75, &rns);
////						}
//						else{
//							RNSStop(&rns);
//							pp.laser = 0;
//							sys.auto2manual = 1;
//							pp.cyon = 0;
//						}
//					}
//					if(ps4.button == CROSS){
//						RNSStop(&rns);
//						pp.cyon = 0;
//						sys.auto2manual = 1;
//						pp.laser = 0;
//						pp.path = 7;
//					}
//				}
//			}
//		}
	}
}

void hitter_mode(void){
	//	initiate cylinder, servoBallH and servoBallV
	BgripOFF
	svBRrealup
	svCloseFlip
	sys.auto2manual = 1;
	MODN.speed = 2.0;
	while(sys.manual == 0){
		if(!LGclose)	LGPW(-9000)
		else			LGPW(0)

		if(ps4.button == CIRCLE){
			while(ps4.button == CIRCLE);
			pp.task = 7;
			pp.path = 7;
			sys.auto2manual = 0;
			RNSSet(&rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
			registerPath(expath, 1, point8);
			RNSPPstart(expath, 1, &rns);
		}
		//led2 - function
		//led3 - path plan
		if(ps4.button == SQUARE){		//to ball rack B
			sys.auto2manual = 0;
			while(ps4.button == SQUARE);
			if(ps4.button == L1_SQUARE){
				while(ps4.button == L1_SQUARE){
					pp.task = 3;
					pp.path = 3;
					pp_func();
				}
			}
			else if(ps4.button == R1_SQUARE){
				while(ps4.button == R1_SQUARE);
				RNSSet(&rns, RNS_RESET_POS);
				RNSSet(&rns, RNS_PP_Reset);
				pp.path = 2;
				pp.task = 2;
				sys.ppstart = 1;
			}
		}
		if(ps4.button == TRIANGLE){
			sys.auto2manual = 0;
			while(ps4.button == TRIANGLE);
			if(ps4.button == L1_TRIANGLE){
				while(ps4.button == L1_TRIANGLE);
				pp.task = 1;
				pp.path = 1;
				pp_func();
			}
			else if(ps4.button == R1_TRIANGLE){
				while(ps4.button == R1_TRIANGLE);
				pp.task = 0;
				pp.path = 0;
				RNSSet(&rns, RNS_RESET_POS);
				RNSSet(&rns, RNS_PP_Reset);
				sys.ppstart = 1;
			}
		}
		if(ps4.button == CROSS){
			RNSStop(&rns);
			sys.auto2manual = 1;
			while(ps4.button == CROSS);
		}

		if(sys.auto2manual){
			if(ps4.button == R1){
				MODN.speed = 5.0;
				while(ps4.button == R1){
					if(ps4.joyR_x >= 0.5){
						ServoMovePulse(&SERVO_BallH, -25);
						Await(250);
					}
					else if(ps4.joyR_x <= -0.5){
						ServoMovePulse(&SERVO_BallH, 25);
						Await(250);
					}
				}
			}

			realMODN(&ps4, &rns);

			if(ps4.button == L1){
				MODN.speed = 2.0;
				if(pp.cyon == 0)
					BgripON
				else
					BgripOFF
				pp.cyon = !pp.cyon;
				while(ps4.button == L1);
			}

			if(ps4.button == DOWN){
				while(ps4.button == DOWN);
				svBRdown
			}

			if(ps4.button == UP){
				while(ps4.button == UP);
				svBRrealup
			}

			if(ps4.button == LEFT){
				while(ps4.button == LEFT);
				svBRB
			}

			if(ps4.button == RIGHT){
				while(ps4.button == RIGHT);
				svBRA
			}

			if(ps4.joyR_y >= 0.5){
				while(ps4.joyR_y >= 0.5);
				svBRup
			}
			if(ps4.joyR_y <= -0.5){
				while(ps4.joyR_y <= -0.5);
				svBallvp(1200);
			}
			if(ps4.joyR_x >= 0.5){
				while(ps4.joyR_x >= 0.5);
				svBRAback
			}
			if(ps4.joyR_x <= -0.5){
				while(ps4.joyR_x <= -0.5);
				svBRBback
			}
		}
		else if(pp.path <= 4 && pp.task <= 4){
			//auto mode
			if(rns.RNS_data.common_instruction == RNS_BUSY){
				led3 = 0;
			}
			else if(sys.ppstart == 1 && (rns.RNS_data.common_instruction != RNS_BUSY)){
				led2 = 1;
				pp.path ++;
				pp_runpath();
				sys.ppstart = 0;
			}
			else if(sys.ppend == 1 && (rns.RNS_data.common_instruction != RNS_BUSY)){
				led3 = 1;
				if(pp.path == 1){
					//stick france
					for(int i = 0; i < 2; i++){
						while (lsl1|| lsl2 || lsr1){
							if((!(lsl2 || lsr1)) && lsl1)
								RNSVelocity(0.00, 1.50, 0.00, 1.00, &rns);
							else if(lsl1 || lsl2)
								RNSVelocity(0.75, 1.25, 0.75, 1.00, &rns);
							else if(lsr1)
								RNSVelocity(-0.75, 1.00, 1.00, -0.75, &rns);
							else{
								RNSStop(&rns);
							}
							if(ps4.button == CROSS){
								RNSStop(&rns);
								sys.auto2manual = 1;
								break;
							}
						}
						Await(150);
						RNSStop(&rns);
					}
				}
				else if(pp.path == 2 && pp.laser == 0){
					RNSStop(&rns);
					sys.auto2manual = 1;
					pp.cyon = 0;
				}
				else if(pp.path == 3){
					//stick france
					for(int i = 0; i < 3; i++){
						while (lsl2|| lsr1 || lsr2){
							if((!(lsl2 || lsr1)) && lsr2)
								RNSVelocity(0.00, 0.00, 1.50, -1.00, &rns);
							else if(lsr1 || lsr2)
								RNSVelocity(-0.75, 0.75, 2.00, -1.75, &rns);
							else if(lsl2)
								RNSVelocity(0.75, 1.00, 1.00, 0.75, &rns);
							else{
								RNSStop(&rns);
							}
							if(ps4.button == CROSS){
								RNSStop(&rns);
								sys.auto2manual = 1;
								break;
							}
						}
						Await(150);
						RNSStop(&rns);
					}
				}
				else if(pp.path == 4 && pp.laser == 0){
					RNSStop(&rns);
					sys.auto2manual = 1;
					pp.cyon = 0;
					pp.path = 7;
				}

				if(ps4.button == CROSS){
					pp.task = 7;
				}
				else{
					pp.task ++;
					pp_func();
				}
				sys.ppend = 0;
			}
			else if(sys.ppend == 0 && sys.ppstart == 0){
				led2 = 0;
			}
		}

		if(pp.path == 0)
			exled1 = 1;
		else if(pp.path == 2)
			exled2 = 1;
		else{
			exled1 = 0;
			exled2 = 0;
		}

		if(!(lsl1))		exled3 = 1;
		else			exled3 = 0;
		if(!(lsl2))		exled4 = 1;
		else			exled4 = 0;
		if(!(lsr1))		exled5 = 1;
		else			exled5 = 0;
		if(!(lsr2))		exled6 = 1;
		else			exled6 = 0;
	}
}

void pp_runpath(void){
	switch(pp.path){
	case 0:
		RNSStop(&rns);
		break;
	case 1:
		svBRup
		RNSSet(&rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
//		RNSPPstart_PS(PP_Points[pp.path - 1], point_num[pp.path - 1], &rns);
		registerPath(ToBallRackA, 3, point1, point2, point3);
		RNSPPstart(ToBallRackA, 3, &rns);
		svBRAback
		Await(4500);
		svBallvp(1200);
		BgripON
//		svBRA
		point8[3] = 90.0;
		break;
	case 2:
		svBRAback
		RNSSet(&rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
		//		RNSPPstart_PS(PP_Points[pp.path - 1], point_num[pp.path - 1], &rns);
//		registerPath(BackToR1A, 4, point5, point6, point7, point8);
//		RNSPPstart(BackToR1A, 4, &rns);
		registerPath(BackToR1A, 4, point5, point6, point7,point8);
		RNSPPstart(BackToR1A, 4, &rns);
		while(SERVO_BallV.currentPulse > svbvrealu_pulse + 150){
			ServoMovePulse(&SERVO_BallV, -6);
			Await(3);
		}
		svBRup
		svBRA
		Await(2000);
		svBRrealup
		laserdir = 0;
		z_tar = 90.0;
//		pp.laser = 1;
		break;
	case 3:
		svBRup
		svBRB
		RNSSet(&rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
//		RNSPPstart_PS(PP_Points[pp.path - 1], point_num[pp.path - 1], &rns);
		registerPath(ToBallRackB, 3, point9, point10, point11);
		RNSPPstart(ToBallRackB, 3, &rns);
		BgripOFF
		Await(4500);
		svBallvp(1200);
		BgripON
		point8[1] = 0.385;
		point8[2] = 0.74;
		point8[3] = 0.0;
		break;
	case 4:
//		svBRBback
		RNSSet(&rns, RNS_PPSetXYZerror, 0.1, 2.0, 0.05, 1.0);
//		RNSPPstart_PS(PP_Points[pp.path - 1], point_num[pp.path - 1], &rns);
//		registerPath(BackToR1B, 3, point12, point4, point13);
//		RNSPPstart(BackToR1B, 3, &rns);
		registerPath(BackToR1B, 4, point14, point12, point4, point13);
		RNSPPstart(BackToR1B, 4, &rns);
		while(SERVO_BallV.currentPulse > svbvrealu_pulse + 150){
			ServoMovePulse(&SERVO_BallV, -6);
			Await(3);
		}
		svBRB
		svBRup
		Await(3000);
		svBRrealup
		laserdir = 0;
		z_tar = 0.0;
//		pp.laser = 1;
		break;
	default:
		pp.path = 0;
		break;
	}
	sys.ppend = 1;
}

void pp_func(void){
	switch(pp.task){
	case 1:
		svBRA
		svBRup
		BgripON
		svBRdown
		Await(550);
		BgripOFF
		Await(550);
		svBallvp(2000)
		break;
	case 3:
		svBRB
		svBRup
		BgripON
		svBRdown
		Await(700);
		BgripOFF;
		Await(700);
		svBallvp(2000)
		break;
	default:
		break;
	}
	sys.ppstart = 1;
}

void setPP_points(void){

	//to back rack A
	point1[0] = 7.6;
	point1[1] = 3.8500;
	point1[2] = 0.600;
	point1[3] = 90.00;
	point1[4] = 0.0;

	point2[0] = 5.0;
	point2[1] = 5.20;
	point2[2] = 3.100;
	point2[3] = 90.00;
	point2[4] = 0.0;

	point3[0] = 2.6;
	point3[1] = 5.33;
	point3[2] = 4.150;
	point3[3] = 90.00;
	point3[4] = 0.0;

	//Back to R1

	point5[0] = 7.5;
	point5[1] = 5.200;
	point5[2] = 1.45;
	point5[3] = 90.00;
	point5[4] = 6.0;

	point6[0] = 7.5;
	point6[1] = 1.70;
	point6[2] = 0.60;
	point6[3] = 90.00;
	point6[4] = 0.0;

	point7[0] = 3.3;
	point7[1] = 0.325;
	point7[2] = 0.520;
	point7[3] = 90.00;
	point7[4] = 0.0;
//-0.175, 0.6
	point8[0] = 2.5;
	point8[1] = 0.125;
	point8[2] = 0.685;
	point8[3] = 90.00;
	point8[4] = 0.0;

	//to ball rack B

	point9[0] = 5.8;
	point9[1] = -4.15;
	point9[2] = 0.7;
	point9[3] = 0;
	point9[4] = 0;
	point9[5] = 0;
	point9[6] = 0;

	point10[0] = 5.5;
	point10[1] = -5.2;
	point10[2] = 3.7;
	point10[3] = 0;
	point10[4] = 0;
	point10[5] = 0;
	point10[6] = 0;

	point11[0] = 2.6;
	point11[1] = -5.50;
	point11[2] = 4.30;
	point11[3] = 0;
	point11[4] = 0;
	point11[5] = 0;
	point11[6] = 0;


	//back to R1
	point14[0] = 3.3;
	point14[1] = -5.35;
	point14[2] = 4.3;
	point14[3] = 0;
	point14[4] = 0;
	point14[5] = 0;
	point14[6] = 0;

	point12[0] = 6.0;
	point12[1] = -5.20;
	point12[2] = 1.65;
	point12[3] = 0;
	point12[4] = 0;
	point12[5] = 0;
	point12[6] = 0;

	point4[0] = 4.5;
	point4[1] = -3.0;
	point4[2] = 0.55;
	point4[3] = 0;
	point4[4] = 0;
	point4[5] = 0;
	point4[6] = 0;

	point13[0] = 3.9;
	point13[1] = -0.175;
	point13[2] = 0.25;
	point13[3] = 0;
	point13[4] = 0;
	point13[5] = 0;
	point13[6] = 0;


	//lagori base
	pLNE[0] = 7.2;
	pLNE[1] = 1.25;
	pLNE[2] = 5.33;
	pLNE[3] = 0.0;
	pLNE[4] = 0.0;

	pLNW[0] = 7.2;
	pLNW[1] = -1.25;
	pLNW[2] = 5.33;
	pLNW[3] = 0.0;
	pLNW[4] = 0.0;

	pLSE[0] = 4.2;
	pLSE[1] = 1.33;
	pLSE[2] = 0.5;
	pLSE[3] = 0.0;
	pLSE[4] = 0.0;

	pLSW[0] = 4.2;
	pLSW[1] = -1.33;
	pLSW[2] = 0.5;
	pLSW[3] = 0.0;
	pLSW[4] = 0.0;
}

void button(void){
	int pulseH = 1000, pulseV = 500, pulseF1 = 1000, pulseF2 = 1000;;
	if(ps4.button == CROSS){
		counter = 0;
		press ++;
		led3 = 0;
		while(ps4.button == CROSS){
			//for long push button 1
			if(counter >= 2000){
				led2 = !led2;
				press= 0;
				while(ps4.button == CROSS);
			}
		}
		led3 = 1;
		switch (press){
		case 1:
			sprintf(data, "get servo pulse of verticle ball(joyR_y)\n");
			UARTSend
			Await(50);
			sprintf(data, "press TRIANGLE/SQUARE to record pulses\n\n");
			UARTSend
			Await(50);
			break;
		case 2:
			sprintf(data, "get servo pulse of horizontal ball(joyR_y)\n");
			UARTSend
			Await(50);
			sprintf(data, "press TRIANGLE/SQUARE to record pulses\n\n");
			UARTSend
			Await(50);
			break;
		case 3:
			sprintf(data, "get servo pulse of lagori flip(joyR_y)\n");
			UARTSend
			Await(50);
			sprintf(data, "press TRIANGLE/SQUARE/CROSS to record pulses\n\n");
			UARTSend
			Await(50);
			break;
		case 4:
			sprintf(data, "get servo pulse of lagori flip(joyR_y)\n");
			UARTSend
			Await(50);
			sprintf(data, "press TRIANGLE/SQUARE/CROSS to record pulses\n\n");
			UARTSend
			Await(50);
			break;
		case 5:
			sprintf(data, "lazer(KF)\n\n");
			UARTSend
			break;
		case 6:
			sprintf(data, "get pulses at ball racks, press and hold L1 for verticle\n");
			UARTSend
			Await(50);
			sprintf(data, "ball pulse, R1 for valve, joyR_y to adj. pulses\n");
			UARTSend
			Await(50);
			sprintf(data, "press TRIANGLE/SQUARE/CROSS(special) to record pulses\n\n");
			UARTSend
			break;
		case 7:
			sprintf(data, "get pulses at lagori sv, press and hold L1 for F2 pulse\n");
			UARTSend
			Await(50);
			sprintf(data, "and gripping, joyR_y to adj. pulses, joyL_x for pw\n");
			UARTSend
			Await(50);
			sprintf(data, "press TRIANGLE/SQUARE/CROSS to record pulses\n\n");
			UARTSend
			break;
		default:
			press = 0;
			break;
		}
	}
	if(ps4.button == CIRCLE){
		led3 = 0;
		while(ps4.button == CIRCLE){
		}
		led3 = 1;
		switch (press){
		case 1:
			exled1 = 1;
			pulse = 1000;
			ServoInitPulseLimit(&SERVO_BallV, 300, 2800);
			while(ps4.button != CIRCLE){
				SERVO_getPulse();
				svBallvp(pulse)
				if(ps4.button == TRIANGLE){
					fx.svbV1 = pulse;
					while(ps4.button == TRIANGLE);
				}
				if(ps4.button == SQUARE){
					fx.svbV2 = pulse;
					while(ps4.button == SQUARE);
				}
				sprintf(data, "pulse: %d\n", pulse);
				UARTSend
			}
			sprintf(data, "BV: %d %d\n", fx.svbV1, fx.svbV2);
			UARTSend
			exled1 = 0;
			while(ps4.button == CIRCLE);
			break;
		case 2:
			exled2 = 1;
			pulse = 1000;
			ServoInitPulseLimit(&SERVO_BallH, 300, 2800);
			while(ps4.button != CIRCLE){
				SERVO_getPulse();
				svBallhp(pulse)
				if(ps4.button == TRIANGLE){
					fx.svbH1 = pulse;
					while(ps4.button == TRIANGLE);
				}
				if(ps4.button == SQUARE){
					fx.svbH2 = pulse;
					while(ps4.button == SQUARE);
				}
				sprintf(data, "pulse: %d\n", pulse);
				UARTSend
			}
			sprintf(data, "BH: %d %d\n", fx.svbH1, fx.svbH2);
			UARTSend
			exled2 = 0;
			while(ps4.button == CIRCLE);
			break;
		case 3:
			pulse = 1000;
			exled3 = 1;
			ServoInitPulseLimit(&SERVO_LagoriF1, 300, 2800);
			while(ps4.button != CIRCLE){
				SERVO_getPulse();
				svLGF1(pulse)
				if(ps4.button == TRIANGLE){
					fx.svlgf1 = pulse;
					while(ps4.button == TRIANGLE);
				}
				if(ps4.button == SQUARE){
					fx.svlgf2 = pulse;
					while(ps4.button == SQUARE);
				}
				if(ps4.button == CROSS){
					fx.svlgf3 = pulse;
					while(ps4.button == CROSS);
				}
				//power window (direction)
				if(ps4.button == L1 && fabs(ps4.joyL_y) >= 0.5)
					LFPW(ps4.joyL_y * -600)
				else if(fabs(ps4.joyR_x) >= 0.5)
					LGPW(ps4.joyL_x * 600)
				sprintf(data, "pulse: %d\n", pulse);
				UARTSend
			}
			sprintf(data, "LF1: %d %d %d\n", fx.svlgf1, fx.svlgf2, fx.svlgf3);
			UARTSend
			exled3 = 0;
			while(ps4.button == CIRCLE);
			break;
		case 4:
			exled4 = 1;
			pulse = 1000;
			ServoInitPulseLimit(&SERVO_LagoriF2, 300, 2800);
			while(ps4.button != CIRCLE){
				SERVO_getPulse();
				svLGF2(pulse)
				if(ps4.button == TRIANGLE){
					fx.svlgf21 = pulse;
					while(ps4.button == TRIANGLE);
				}
				if(ps4.button == SQUARE){
					fx.svlgf22 = pulse;
					while(ps4.button == SQUARE);
				}
				if(ps4.button == CROSS){
					fx.svlgf23 = pulse;
					while(ps4.button == CROSS);
				}
				//power window (direction)
				if(ps4.button == L1 && fabs(ps4.joyL_y) >= 0.5)
					LFPW(ps4.joyL_y * -600)
				else if(fabs(ps4.joyL_x) >= 0.5)
					LGPW(ps4.joyL_x * 600)
				sprintf(data, "pulse: %d\n", pulse);
				UARTSend
			}
			sprintf(data, "LF2: %d %d %d\n", fx.svlgf21, fx.svlgf22, fx.svlgf23);
			UARTSend
			exled4 = 1;
			while(ps4.button == CIRCLE);
			break;
		case 5:
			exled5 = 1;
			while(ps4.button != CIRCLE){
				sprintf(data, "kfx=%.3f, kfy=%.3f, x=%.3f, y=%.3f\n", x_kfo, y_kfo, laser_dist_x, laser_dist_y);
				UARTSend
				Await(50);
			}
			exled5 = 0;
			while(ps4.button == CIRCLE);
			break;
		case 6:
			exled1 = 1;
			exled2 = 1;
			pulseV = 1000;
			pulseH = 1000;
			ServoInitPulseLimit(&SERVO_BallV, 300, 2800);
			ServoInitPulseLimit(&SERVO_BallH, 300, 2800);
			while(ps4.button != CIRCLE){
				if(ps4.button == L1 && ps4.joyR_y >= 0.75)			pulseV += 25;
				else if(ps4.button == L1 && ps4.joyR_y <= -0.75)	pulseV -= 25;
				else if(ps4.joyR_y >= 0.75)							pulseH += 25;
				else if(ps4.joyR_y <= -0.75)						pulseH -= 25;

				if(pulseV < 300)	pulseV = 300;
				if(pulseV > 2800)	pulseV = 2800;
				if(pulseH < 300)	pulseH = 300;
				if(pulseH > 2800)	pulseH = 2800;

				if(ps4.button == TRIANGLE){
					fx.svbV1 = pulseV;
					fx.svbH1 = pulseH;
					while(ps4.button == TRIANGLE);
				}
				if(ps4.button == SQUARE){
					fx.svbV2 = pulseV;
					fx.svbH2 = pulseH;
					while(ps4.button == SQUARE);
				}
				if(ps4.button == CROSS){
					fx.svbV1 = pulseV;
					while(ps4.button == CROSS);
				}
				if(ps4.button == R1){
					while(ps4.button == R1);
					led3 = !led3;
					if(!fx.cyapp)
						BgripON
					else
						BgripOFF
						fx.cyapp = !fx.cyapp;
				}
				svBallhp(pulseH)
				svBallvp(pulseV)
				Await(150);
			}
			sprintf(data, "BV: %d %d,  BH: %d, %d\n", fx.svbV1, fx.svbV2, fx.svbH1, fx.svbH2);
			UARTSend
			Await(20);
			sprintf(data, "%d, %d\n", enqpwLG, enqpwLlf);
			UARTSend
			exled1 = 0;
			exled2 = 0;
			while(ps4.button == CIRCLE);
			break;
		case 7:
			exled3 = 1;
			exled4 = 1;
			pulseF1 = 1000;
			pulseF2 = 1650;
			ServoInitPulseLimit(&SERVO_LagoriF1, 300, 2800);
			ServoInitPulseLimit(&SERVO_LagoriF2, 300, 2800);
			while(ps4.button != CIRCLE){
				if(ps4.button == L1 && ps4.joyR_y >= 0.75)			pulseF2 += 25;
				else if(ps4.button == L1 && ps4.joyR_y <= -0.75)	pulseF2 -= 25;
				else if(ps4.joyR_y >= 0.75)							pulseF1 += 25;
				else if(ps4.joyR_y <= -0.75)						pulseF1 -= 25;

				if(pulseF1 < 300)	pulseF1 = 300;
				if(pulseF1 > 2800)	pulseF1 = 2800;
				if(pulseF2 < 300)	pulseF2 = 300;
				if(pulseF2 > 2800)	pulseF1 = 2800;

				if(ps4.button == TRIANGLE){
					fx.svlgf1 = pulseF1;
					fx.svlgf21 = pulseF2;
					while(ps4.button == TRIANGLE);
				}
				if(ps4.button == SQUARE){
					fx.svlgf2 = pulseF1;
					fx.svlgf22 = pulseF2;
					while(ps4.button == SQUARE);
				}
				if(ps4.button == CROSS){
					fx.svlgf3 = pulseF1;
					fx.svlgf23 = pulseF2;
					while(ps4.button == CROSS);
				}

				//power window (direction)
				if(ps4.button == L1 && fabs(ps4.joyL_x) >= 0.5)
					LGPW(ps4.joyL_x * -6000)
				else if(fabs(ps4.joyL_y) >= 0.5)
					LFPW(ps4.joyL_y * 6000)
				else{
					LGPW(0)
					LFPW(0)
				}

				svLGF1(pulseF1)
				svLGF2(pulseF2)
				sprintf(data, "%d %d\n", pulseF1, pulseF2);
				UARTSend
				Await(150);
			}
			sprintf(data, "LF1: %d %d %d\n", fx.svlgf1, fx.svlgf2, fx.svlgf3);
			UARTSend
			Await(50);
			sprintf(data, "LF2: %d %d %d\n", fx.svlgf21, fx.svlgf22, fx.svlgf23);
			UARTSend
			Await(50);
			exled3 = 0;
			exled4 = 0;
			while(ps4.button == CIRCLE);
			break;
		}
		press = 0;
	}
}

void limit_sw(void){
	if(!(lsLGl))
		exled1 = 1;
	else if(!(lsLGr))
		exled2 = 1;
	else if(!(lsl1))
		exled3 = 1;
	else if(!(lsl2))
		exled4 = 1;
	else if(!(lsr1))
		exled5 = 1;
	else if(!(lsr2))
		exled6 = 1;
	else{
		exled1 = 0;
		exled2 = 0;
		exled3 = 0;
		exled4 = 0;
		exled5 = 0;
		exled6 = 0;
	}
}

void analog_sr(void){
	if(LGclose)
		exled1 = 1;
	if(LGopen)
		exled2 = 1;
	if(LGHms)
		exled3 = 1;
	if(LGHm)
		exled4 = 1;
	if(LGHml)
		exled5 = 1;
	if(LGHl)
		exled6 = 1;
	if(LGbasel){
		exled1 = 1;
		exled2 = 1;
	}
	if(LGbaser){
		exled3 = 1;
		exled4 = 1;
	}
	Await(20);
	exled1 = 0;
	exled2 = 0;
	exled3 = 0;
	exled4 = 0;
	exled5 = 0;
	exled6 = 0;
	Await(20);
}

void functest(void){
	//NAVI
	if(ps4.button == R1)
		MODN.speed = 7.0;
	else if(ps4.button == L1)
		MODN.speed = 2.0;
	else
		MODN.speed = 4.5;
	realMODN(&ps4, &rns);

	//power window (direction)
	if(ps4.button == L1 && fabs(ps4.joyR_x) >= 0.5)
		LGPW(ps4.joyR_x * 6000)
	else if(ps4.joyR_y >= 0.5)
		LFPW(ps4.joyR_y * 7500)
	else if(ps4.joyR_y <= -0.5)
		LFPW(ps4.joyR_y * 6000)
	else{
		LFPW(0)
		LGPW(0)
	}

	if(ps4.button == OPTION){
		n = 8;
		enq();
		Await(200);
	}

	if(ps4.button == SHARE){
		QEIReset(QEI1);
		QEIReset(QEI4);
	}

	//test SERVO at ball gripper
	if(ps4.button == UP){
		while(ps4.button == UP);
		led3 = !led3;
		if(!fx.svapp){
			svBRup
		}
		else{
			svBRdown
		}
		fx.svapp = !fx.svapp;
	}
	else if(ps4.button == LEFT){
		while(ps4.button == LEFT);
		led3 = !led3;
		if(!fx.svapp){
			svBRA
		}
		else{
			svBRB
		}
		fx.svapp = !fx.svapp;
	}

	//test flipping servo
	else if(ps4.button == DOWN){
		svNoFlip
		while(ps4.button == DOWN);
		if(ps4.button == L1_DOWN){
			led3 = !led3;
			if(!fx.svapp){
				svCloseFlip
			}
			else{
				svFlip
			}
			fx.svapp = !fx.svapp;
		}
	}

	//test ball cylinder
	if(ps4.button == TRIANGLE){
		while(ps4.button == TRIANGLE);
		led3 = !led3;
		if(!fx.cyapp)
			BgripOFF
		else
			BgripON
		fx.cyapp = !fx.cyapp;
	}
}

void SERVO_getPulse(void){
	if(ps4.joyR_y >= 0.75){
		pulse += 25;
	}
	if(ps4.joyR_y <= -0.75){
		pulse -= 25;
	}
	Await(100);
}

void KFilterAdjust(void){
	static float avg_x = 0, avg_y = 0;
	static float kf[2][3]={{7.27, 16.03, 0.15}, {7.5, 12.01, 2.0}};
	static int arr_x = 0, arr_y = 0, num = 0, n = 0;
	while(ps4.button == TRIANGLE){
		avg_x = (avg_x * n + adc.ADC_value[0]) / (n + 1);
		avg_y = (avg_y * n + adc.ADC_value[1]) / (n + 1);
		sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f\n", avg_x, x_kfo, avg_y, y_kfo, y_kfoav);
//		sprintf(data, "%d, %.3f, %.3f\n", adc.ADC_value[0], x_kfo, avg_x);
		UARTSend
		led2 = !led2;
		Await(20);
		n++;
	}
	if(ps4.button == SQUARE){
		while(ps4.button == SQUARE);
		for(int i = 0; i < 2; i++){
			for(int j = 0; j < 3; j++){
				sprintf(data, "%.3f ", kf[i][j]);
				UARTSend
			}
		}
		sprintf(data, "kf\n");
		UARTSend
	}
	if(ps4.button == CIRCLE){
		while(ps4.button == CIRCLE);
		avg_x = 0;
		avg_y = 0;
		n = 0;
		KalmanFilterInit(1, &(adc.ADC_value[0]), &x_kfo, kf[0][0], kf[0][1], kf[0][2], &kf_adc_x);
		KalmanFilterInit(1, &(adc.ADC_value[1]), &y_kfo, kf[1][0], kf[1][1], kf[1][2], &kf_adc_y);
		Await(50);
	}
	if(ps4.button == CROSS){
		while(ps4.button == CROSS){}
		kf[arr_y][arr_x] = kf[arr_y][arr_x] + num * 0.01;
		sprintf(data, "y = %d, x = %d, kf = %f\n", arr_y, arr_x, kf[arr_y][arr_x]);
		UARTSend
		num = 0;
	}
	if(ps4.button == UP){
		while(ps4.button == UP){}
		arr_y ++;
		if(arr_y > 1)	arr_y = 0;
		sprintf(data, "y = %d\n", arr_y);
		UARTSend
	}
	if(ps4.button == RIGHT){
		while(ps4.button == RIGHT){}
		arr_x ++;
		if(arr_x > 2)	arr_x = 0;
		sprintf(data, "x = %d\n", arr_x);
		UARTSend
	}
	if(ps4.button == L1){
		while(ps4.button == L1){}
		num --;
		sprintf(data, "num = %.3f\n", (float)num/50);
		UARTSend
	}
	if(ps4.button == R1){
		while(ps4.button == R1){}
		num ++;
		sprintf(data, "num = %.3f\n", (float)num/50);
		UARTSend
	}
	if(ps4.joyR_y > 0.5){
		num ++;
		sprintf(data, "num = %.3f\n", (float)num/50);
		UARTSend
		Await(10);
	}
	if(ps4.joyR_y < -0.5){
		num --;
		sprintf(data, "num = %.3f\n", (float)num/50);
		UARTSend
		Await(10);
	}
}

void enq (void){
	switch(n){
	case 1:
		sprintf(data, "short press SQUARE to set 12.5cm\n");
		UARTSend
		Await(50);
		sprintf(data, "or long press SQUARE (>2s) to set 15cm\n");
		UARTSend
		Await(50);
		while(ps4.button != SQUARE);
		uint32_t tick = HAL_GetTick();
		while(ps4.button == SQUARE){
			sprintf(data, "time: %.3f\n", (float)(HAL_GetTick() - tick) / 1000);
			UARTSend
			Await(250);
		}
		diameter = (HAL_GetTick() - tick > 2000)? 0.150: 0.125;
		sprintf(data, "diameter = %f\n", diameter);
		UARTSend
		Await(20);
		sprintf(data, "2s later start get pulse\n");
		UARTSend
		Await(20);
		RNSSet(&rns, RNS_F_KCD_PTD, 1.0, diameter * 3.142 / 1.0);
		RNSSet(&rns, RNS_B_KCD_PTD, 1.0, diameter * 3.142 / 1.0);
		RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 1.0 , 19999.0);
		RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 1.0, 19999.0);
		RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 1.0, 19999.0);
		RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 1.0, 19999.0);
		RNSSet(&rns, RNS_F_LEFT_VEL_PID,  1.5, 1.0, 0.0);
		RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 1.5, 1.0, 0.0);
		RNSSet(&rns, RNS_B_LEFT_VEL_PID,  1.5, 1.0, 0.0);
		RNSSet(&rns, RNS_B_RIGHT_VEL_PID,  1.5, 1.0, 0.0);
		Await(2000);
		n = 2;
		break;

	case 2:
		RNSSet(&rns, RNS_RESET_POS);
		while(ps4.button != SQUARE){
			RNSEnquire(RNS_POS_BOTH, &rns);
			pa = rns.enq.enq_buffer[0].data;
			pb = rns.enq.enq_buffer[1].data;
			pc = rns.enq.enq_buffer[2].data;
			pd = rns.enq.enq_buffer[3].data;
			sprintf(data, "pulse: %.3f, %.3f, %.3f, %.3f\n", pa, pb, pc, pd);
			UARTSend
			Await(20);
		}
		while(ps4.button == SQUARE);
		RNSSet(&rns, RNS_F_KCD_PTD, pa / pb, diameter * 31.4159 / pa);
		RNSSet(&rns, RNS_B_KCD_PTD, pc / pd, diameter * 31.4159 / pc);
		Await(600);
		sprintf(data, "kcd & ptd set, press SQUARE to get the max velocity\n");
		UARTSend
		Await(20);
		n = 3;
		break;

	case 4:
		RNSSet(&rns, RNS_RESET_POS);
		while(ps4.button != SQUARE){
			RNSEnquire(RNS_VEL_BOTH, &rns);
			a = rns.enq.enq_buffer[0].data;
			b = rns.enq.enq_buffer[1].data;
			c = rns.enq.enq_buffer[2].data;
			d = rns.enq.enq_buffer[3].data;
			if(a > v1)		v1 = a;
			if(b > v2)		v2 = b;
			if(c > v3)		v3 = c;
			if(d > v4)		v4 = d;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f\n", v1, v2, v3, v4);
			UARTSend
			Await(20);
		}
		while(ps4.button == SQUARE);
		RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / v1, 19999.0);
		RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / v2, 19999.0);
		RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / v3, 19999.0);
		RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / v4, 19999.0);
		Await(600);
		sprintf(data, "max velocity set complete\n");
		UARTSend
		Await(20);
		sprintf(data, "diameter = %f\n", diameter);
		UARTSend
		Await(20);
		sprintf(data, "pulse: %.3f, %.3f, %.3f, %.3f\n", pa, pb, pc, pd);
		UARTSend
		Await(20);
		sprintf(data, "max velocity: %.3f, %.3f, %.3f, %.3f\n", v1, v2, v3, v4);
		UARTSend
		n = 0;
		break;

//	case 3:
//		RNSEnquire(RNS_X_Y_POS, &rns);
//		xpos = rns.enq.enq_buffer[0].data;
//		ypos = rns.enq.enq_buffer[1].data;
//		z = rns.enq.enq_buffer[2].data;
//		break;

	case 6:
		RNSSet(&rns, RNS_RESET_POS);
		while(ps4.button != SQUARE){
			RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
			xpos = rns.enq.enq_buffer[0].data;
			ypos = rns.enq.enq_buffer[1].data;
			z = rns.enq.enq_buffer[2].data;
			zrad = rns.enq.enq_buffer[3].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f\n", xpos, ypos, z, zrad);
			UARTSend
			Await(20);
		}
		while(ps4.button == SQUARE);
		n = 0;
		break;

	case 8:
		sprintf(data, "%d, %d\n", enqpwLG, enqpwLlf);
		UARTSend
		while(ps4.button == SQUARE);
		n = 0;
		break;

//	case 10:
//		RNSEnquire(RNS_VEL_BOTH, &rns);
//		a = rns.enq.enq_buffer[0].data;
//		b = rns.enq.enq_buffer[1].data;
//		c = rns.enq.enq_buffer[2].data;
//		d = rns.enq.enq_buffer[3].data;
//		sprintf(data, "%.3f, %.3f, %.3f, %.3f\n", a, b, c, d);
//		UARTSend
//		n = 0;
//		break;
	}
}

void Await(uint32_t ticks){
	counter = 0;
	while(counter < ticks);
}

Tune_Int_t lcdIntList[NUM_INT_UPDATE]={
		//Example: float a = 1.0;
		//		varI(ps4.button, 0, 0),
};

Tune_Float_t lcdFloatList[NUM_FLOAT_UPDATE]={
		//		varF(ps4.joyR_x, 0, 0),
};


void ILI9341_Init_List(void){
	ILI9341row = 5;

	for(int i=0; i<NUM_INT_UPDATE; i++){
		ILI9341_Draw_String(5, ILI9341row, WHITE, BLACK, lcdIntList[i].varName, 2);
		ILI9341row += 15;
	}

	for(int i=0; i<NUM_FLOAT_UPDATE; i++){
		ILI9341_Draw_String(5, ILI9341row, WHITE, BLACK, lcdFloatList[i].varName, 2);
		ILI9341row += 15;
	}
}

void ILI9341_Update_List(void){
	//Don't update freqeuntly as this function eat around 30ms even with spi dma transmit
	int updateRow=5;
	char lcdData[100];
	for(int i=0; i<NUM_INT_UPDATE; i++){
		sprintf(lcdData, "%8d", *(lcdIntList[i].ptr));
		ILI9341_Draw_String(110, updateRow, WHITE, BLACK, lcdData, 2);
		updateRow += 15;
	}

	for(int i=0; i<NUM_FLOAT_UPDATE; i++){
		sprintf(lcdData, "%8.3f", *(lcdFloatList[i].ptr));
		ILI9341_Draw_String(110, updateRow, WHITE, BLACK, lcdData, 2);
		updateRow += 15;
	}
}
