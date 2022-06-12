/*******************************************************************************
 * Title   : adapter.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: includes all the important includes and pin definitions
 *
 * Version History:
 *  1.0 - converted to HAL library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef SRC_ADAPTER_H_
#define SRC_ADAPTER_H_


/* Private variables ---------------------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "BIOS/bios.h"
#include <math.h>
#include "CAN/can.h"
#include "MODN/MODN.h"
#include "PID/PID.h"
#include "ABT/ABT.h"
#include "I2C/i2c.h"
#include "SPI/SPI.h"
#include "PSx_Interface/PSx_Interface.h"
#include "RNS_interface/RNS_interface.h"
#include "ADC/adc.h"
#include "SERVO/servo.h"
#include "KF/KF.h"
#include "LASER/laser.h"
#include "STEPPER/stepper.h"
#include "SERVO_DRIVER/servo_driver.h"
#include "Moving_Average/mov_ave.h"
#include "VESC_CAN/vesc_interface.h"
#include "Eeprom/eeprom.h"
#include "ILI9341/ILI9341_Driver.h"
#include "Dongbei/dongbei.h"
#include "RGB/rgb.h"
#include "TuningInterface/TuningInterface.h"
#include "Tune.h"
#include "ESP32/esp32.h"
#include "TFmini/tfmini.h"
/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define LED1_PIN			GPIOC, GPIO_PIN_13
#define LED2_PIN			GPIOC, GPIO_PIN_14
#define LED3_PIN			GPIOC, GPIO_PIN_15

#define PB1_PIN				GPIOB, GPIO_PIN_7
#define PB2_PIN				GPIOE, GPIO_PIN_0

#define IP1_PIN				GPIOE, GPIO_PIN_12              /* TIM1_CH3N                                              */
#define IP2_PIN				GPIOE, GPIO_PIN_13              /* TIM1_CH3                                               */
#define IP3_PIN				GPIOE, GPIO_PIN_14              /* TIM1_CH4                                               */
#define IP4_PIN				GPIOE, GPIO_PIN_15              /* TIM1_BKN                                               */
#define IP5_PIN				GPIOB, GPIO_PIN_14              /* TIM12_CH1                                              */
#define IP6_PIN				GPIOB, GPIO_PIN_15              /* TIM12_CH2                                              */
#define IP7_PIN				GPIOD, GPIO_PIN_10              /*                                                        */
#define IP8_PIN				GPIOD, GPIO_PIN_11              /*                                                        */
#define IP9_PIN				GPIOC, GPIO_PIN_8               /* TIM8_CH3                                               */
#define IP10_PIN			GPIOA, GPIO_PIN_10              /* TIM1_CH3                                               */
#define IP11_PIN			GPIOD, GPIO_PIN_3               /*                                                        */
#define IP12_PIN			GPIOD, GPIO_PIN_4               /*                                                        */
#define IP13_PIN			GPIOD, GPIO_PIN_7               /*                                                        */
#define IP14_PIN			GPIOB, GPIO_PIN_5               /*                                                        */
#define IP15_PIN			GPIOB, GPIO_PIN_6               /* TIM4_CH1                                               */

#define IP16_Analog1_PIN	GPIOC, GPIO_PIN_0
#define IP17_Analog2_PIN	GPIOC, GPIO_PIN_1
#define IP18_Analog3_PIN	GPIOC, GPIO_PIN_2
#define IP19_Analog4_PIN	GPIOC, GPIO_PIN_3
#define IP20_Analog5_PIN	GPIOC, GPIO_PIN_4
#define IP21_Analog6_PIN	GPIOC, GPIO_PIN_5

#define QEI1_PLUSEA_PIN		GPIOE , GPIO_PIN_9
#define QEI1_PLUSEB_PIN		GPIOE , GPIO_PIN_11

#define QEI4_PLUSEA_PIN		GPIOD , GPIO_PIN_12
#define QEI4_PLUSEB_PIN		GPIOD , GPIO_PIN_13

#define QEI6_PLUSEA_PIN		GPIOC , GPIO_PIN_6
#define QEI6_PLUSEB_PIN		GPIOC , GPIO_PIN_7

#define TIM5_CHANNEL1_PIN	GPIOA, GPIO_PIN_0
#define TIM5_CHANNEL2_PIN	GPIOA, GPIO_PIN_1
#define TIM5_CHANNEL3_PIN	GPIOA, GPIO_PIN_2
#define TIM5_CHANNEL4_PIN	GPIOA, GPIO_PIN_3

#define TIM9_CHANNEL1_PIN	GPIOE, GPIO_PIN_5
#define TIM9_CHANNEL2_PIN	GPIOE, GPIO_PIN_6

#define TIM3_ChANNEL1_PIN   GPIOA, GPIO_PIN_6
#define TIM3_CHANNEL3_PIN	GPIOB, GPIO_PIN_0
#define TIM3_CHANNEL4_PIN	GPIOB, GPIO_PIN_1

#define MUX1_INPUT_PIN 		GPIOE , GPIO_PIN_1
#define MUX1_S0_PIN 		GPIOE , GPIO_PIN_2
#define MUX1_S1_PIN 		GPIOE , GPIO_PIN_3
#define MUX1_S2_PIN 		GPIOE , GPIO_PIN_4

#define SR_SCK_PIN			GPIOE , GPIO_PIN_7
#define SR_RCK_PIN			GPIOE , GPIO_PIN_8
#define SR_SI_PIN			GPIOE , GPIO_PIN_10

#define SPI1_NSS_PIN		GPIOA, GPIO_PIN_4
#define SPI1_SCK_PIN		GPIOA, GPIO_PIN_5
#define SPI1_MISO_PIN		GPIOA, GPIO_PIN_6
#define SPI1_MOSI_PIN		GPIOA, GPIO_PIN_7

#define UART2_Tx			GPIOD , GPIO_PIN_5
#define UART2_Rx			GPIOD , GPIO_PIN_6

#define UART3_Tx			GPIOD , GPIO_PIN_9
#define UART3_Rx			GPIOD , GPIO_PIN_8

#define UART4_Tx			GPIOC , GPIO_PIN_10
#define UART4_Rx			GPIOC , GPIO_PIN_11

#define UART5_Tx			GPIOC , GPIO_PIN_12
#define UART5_Rx			GPIOD , GPIO_PIN_2

#define CAN1_Tx				GPIOD , GPIO_PIN_1
#define CAN1_Rx				GPIOD , GPIO_PIN_0

#define CAN2_Tx				GPIOB , GPIO_PIN_13
#define CAN2_Rx				GPIOB , GPIO_PIN_12

MUX_t MUX;
shiftreg_t SR;
RNS_interface_t rns;
BDC_t BDC1, BDC2, BDC3, BDC4, BDC5, BDC6, BDC7, BDC8;
uint8_t insData_receive[2];
PSxBT_t ps4;
ABT_t filter;
ADC_t adc;
LASER_t X_laser, Y_laser;
KALMANFILTER_t kf_adc_x, kf_adc_y, kf_pres;
Mov_Ave_t move_aveX, move_aveY;
//PID_t pid_laser_X,pid_laser_Y,pid_pres,pid_z;
tfmini_t tfmini;
Srv_Drv_t srv_drv;
Mov_Ave_t mov_l_r,mov_l_l;
VESC_t vesc1;
//SERVO_t vesc_ppm[3], servo, servo1;
SERVO_t SERVO_BallV, SERVO_BallH, SERVO_LagoriF1, SERVO_LagoriF2;

#define PB1 		GPIOB_IN->bit7
#define PB2 		GPIOE_IN->bit0

#define led1		GPIOC_OUT->bit13
#define led2		GPIOC_OUT->bit14
#define led3		GPIOC_OUT->bit15
	
//extraLED
#define exled1		GPIOD_OUT->bit9
#define exled2		GPIOD_OUT->bit8
#define exled3		GPIOA_OUT->bit7
#define exled4		GPIOA_OUT->bit6
#define exled5		GPIOA_OUT->bit5
#define exled6		GPIOA_OUT->bit4

// HSPM
//#define BgripON 	SR.cast[1].bit3 = 1;	//HSPM3 S1
//#define BgripOFF 	SR.cast[1].bit3 = 0;
#define BgripON 	GPIOC_OUT->bit8 = 1;	//IP9
#define BgripOFF 	GPIOC_OUT->bit8 = 0;
#define LGPW(a) 	WriteBDC(&BDC1, -a);
#define LFPW(a) 	WriteBDC(&BDC2, -a);

//limit switch
#define lsLGl		IP10 == 1
#define lsLGr		IP11 == 1
#define lsl1		IP12 == 1
#define lsl2		IP13 == 1
#define lsr1		IP14 == 1
#define lsr2		IP15 == 1

//analog sensor	set to high
#define LGopen			IP1 == 0
#define LGclose			IP2 == 0
#define LGHms			IP3 == 0
#define LGHm			IP4 == 0
#define LGHml			IP5 == 0
#define LGHl			IP6 == 0
#define LGbasel			IP7 == 0
#define LGbaser			IP8 == 0

//SERVO
#define svBallvp(a)		ServoSetPulse(&SERVO_BallV, a);
#define svBallhp(a) 	ServoSetPulse(&SERVO_BallH, a);
#define svLGF1(a)		ServoSetPulse(&SERVO_LagoriF1, a);
#define svLGF2(a)		ServoSetPulse(&SERVO_LagoriF2, a);

#define svBRrealup		svBallvp(svbvrealu_pulse);
#define svBRup			svBallvp(svbvu_pulse);
#define svBRdown		svBallvp(svbvd_pulse);
#define svBRA			svBallhp(svbhA_pulse);
#define svBRAback		svBallhp(svbhA2_pulse);
#define svBRB			svBallhp(svbhB_pulse);
#define svBRBback		svBallhp(svbhB2_pulse);
#define svCloseFlip		svLGF1(svloff); svLGF2(svloff2);
#define svFlip 			svLGF1(svlf); 	svLGF2(svlf2);
#define svNoFlip 		svLGF1(svlnf); 	svLGF2(svlnf2);

//#define svbhA_pulse	2000
//#define svbhB_pulse	725
//#define svbhA2_pulse	2100
//#define svbhB2_pulse	700
#define svbhA_pulse		2100
#define svbhB_pulse		803
#define svbhA2_pulse	2166
#define svbhB2_pulse	750
#define svbvrealu_pulse	425
#define svbvu_pulse		600
#define svbvd_pulse		2550

#define svloff			1950
#define svlf			550
#define svlnf			1025
#define svloff2			900
#define svlf2			2266
#define svlnf2			1775

//pulley Power Window
#define layerG			500
#define layer1			7500
#define layer2			13000
#define layer3			17500
#define layer4			21000
#define layer5			22100

#define svfs_layer		3400
#define svfms_layer		4700
#define svfm_layer		5600
#define svfml_layer		6600
#define svfl_layer		7000

//lagori size enq
#define dangerclose		100
//#define dangerclose		-50
#define LGsize2			300
#define LGsize3			717
#define LGsize4			1001
#define LGsize5			1583

//#define PPArgSet		45.0
#define UARTSend 		UARTPrintString(&huart2, data);
#define z_err			6.6 	//0.10rad
#define fAdist 			6.26
#define fBdist			7.0
#define R1dist			0.04

int enqpwLG, enqpwLlf;
//path plan
float ToBallRackA[3][7];
float BackToR1A[4][7];
float ToBallRackB[4][7];
float BackToR1B[4][7];
float LBaseN[3][7];
float LBaseS[3][7];
float LBaseE[3][7];
float LBaseW[3][7];

//Global Declarations
float v1, v2, v3, v4, wa, xr, yr;         //MODN variables
float xpos, ypos, z, zrad, diameter;      //Encoder Values
float a, b, c, d, pa, pb, pc, pd;		  //Enquiry Values
volatile uint16_t  adc1_buf[7];
volatile uint16_t  adc2_buf[7];
volatile uint16_t  adc3_buf[7];

int counter, ledtick, lagoricounter;                     //global timer variable
float speed, vz, x_err, y_err, z_tar;
float laser_dist_x, laser_dist_y;
float x_kfo, y_kfo, x_kfoav, y_kfoav;
float pidi_laser_X, pido_laser_X, pidi_laser_Y, pido_laser_Y;

int servo_lagori_val[5][2];
int servo_init_val[5];

int realPulse;

enum{
	F_dir,
	R_dir,
	B_dir,
	L_dir
}orientation;

union{
	float data;
	struct{
		char byte1;
		char byte2;
		char byte3;
		char byte4;
	};
}buf1_receive[2];
union{
	float data;
	struct{
		char byte1;
		char byte2;
		char byte3;
		char byte4;
	};
}buf2_receive[2];

typedef enum{
	RNS_PACKET,
	VESC_PACKET,
}PACKET_t;

uint32_t faul_counter;
struct{
	uint16_t statusword;
	int pos_act;
	int vel_act;
	int16_t tor_act;
	uint16_t ODindex;
	uint8_t ODsubindex;
	uint8_t num_valid;
	uint8_t buffer[4];
	int HomeOffset;
	uint32_t velKp;
	int8_t modeOperation;
	union{
		uint16_t flags;
		struct{
			unsigned can    	    :1;
			unsigned flag1          :1;
			unsigned flag2    		:1;
			unsigned flag3          :1;
			unsigned flag4          :1;
			unsigned flag5          :1;
			unsigned flag6          :1;
			unsigned flag7          :1;
			unsigned flag8          :1;
			unsigned flag9          :1;
			unsigned flag10         :1;
			unsigned flag11         :1;
			unsigned flag12         :1;
			unsigned flag13 	    :1;
			unsigned flag14         :1;
			unsigned flag15        	:1;
		};
	};
}Faul_t;
void CAN_PROCESS(PACKET_t packet_src);
void Initialize (void);

#ifdef __cplusplus
}
#endif
#endif /* SRC_ADAPTER_H_ */
