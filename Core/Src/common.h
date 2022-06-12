/*******************************************************************************
 * Title   : common.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: Sensor and function definitions
 *
 * Version History:
 *  1.0 - converted to hal library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"
#include "cmsis_os.h"
#include "ROS_Interface/ROS_Interface.h"

#define IP1  	HAL_GPIO_ReadPin(IP1_PIN)
#define IP2  	HAL_GPIO_ReadPin(IP2_PIN)
#define IP3  	HAL_GPIO_ReadPin(IP3_PIN)
#define IP4		HAL_GPIO_ReadPin(IP4_PIN)
#define IP5 	HAL_GPIO_ReadPin(IP5_PIN)
#define IP6 	HAL_GPIO_ReadPin(IP6_PIN)
#define IP7		HAL_GPIO_ReadPin(IP7_PIN)
#define IP8	 	HAL_GPIO_ReadPin(IP8_PIN)
#define IP9  	HAL_GPIO_ReadPin(IP9_PIN)
#define IP10    HAL_GPIO_ReadPin(IP10_PIN)
#define IP11  	HAL_GPIO_ReadPin(IP11_PIN)
#define IP12 	HAL_GPIO_ReadPin(IP12_PIN)
#define IP13  	HAL_GPIO_ReadPin(IP13_PIN)
#define IP14 	HAL_GPIO_ReadPin(IP14_PIN)
#define IP15	HAL_GPIO_ReadPin(IP15_PIN)

//ANALOG PIN//
#define IP16	HAL_GPIO_ReadPin(IP16_Analog1_PIN)
#define IP17	HAL_GPIO_ReadPin(IP17_Analog2_PIN)
#define	IP18 	HAL_GPIO_ReadPin(IP18_Analog3_PIN)
#define IP19	HAL_GPIO_ReadPin(IP19_Analog4_PIN)
#define IP20	HAL_GPIO_ReadPin(IP20_Analog5_PIN)
#define IP21	HAL_GPIO_ReadPin(IP21_Analog6_PIN)

#define Mux1	MUX.mux_data.bit0
#define Mux2	MUX.mux_data.bit1
#define Mux3	MUX.mux_data.bit2
#define Mux4	MUX.mux_data.bit3
#define Mux5	MUX.mux_data.bit4
#define Mux6	MUX.mux_data.bit5
#define Mux7	MUX.mux_data.bit6
#define Mux8	MUX.mux_data.bit7

osThreadId_t MainTaskHandle;
osThreadId_t EmergencyTaskHandle;
osThreadId_t TuneTaskHandle;
osThreadId_t LagoriTaskHandle;
osThreadId_t BLDCTaskHandle;
//osThreadId_t LaserNavigateTaskHandle;
//osThreadId_t SecondaryTaskHandle;
//osThreadId_t FlywheelPitchPIDTaskHandle;
//osThreadId_t FlywheelYawPIDTaskHandle;
//osThreadId_t TestTaskHandle;

osSemaphoreId_t TuneSemaphore;

int n, press, pulse, target_enq;
float laserdir;

typedef union{
	uint32_t flags;
	struct{
		//Least significant 16 bits can be cleared all at once by
		//sys.flags = 0 for example during emergency
		unsigned control      :1;
		unsigned tunePid      :1;
		unsigned tunePidex    :1;
		unsigned tunePP		  :1;
		unsigned manual       :1;
		unsigned auto2manual  :1;
		unsigned ppstart      :1;
		unsigned ppend		  :1;
		unsigned stickf       :1;
		unsigned flag9        :1;
		unsigned flag10       :1;
		unsigned flag11       :1;
		unsigned flag12       :1;

		//Most significant 16 bits are not clear

		unsigned flag17       :1;
		unsigned flag18       :1;
		unsigned flag19       :1;
		unsigned flag20       :1;
		unsigned flag21       :1;
		unsigned flag22       :1;
		unsigned flag23		  :1;
		unsigned flag24       :1;
		unsigned flag25       :1;
		unsigned flag26	      :1;
		unsigned flag27		  :1;
		unsigned flag28		  :1;
		unsigned flag29		  :1;
		unsigned flag30		  :1;
		unsigned flag31		  :1;

		//A flag can use more than 1 bit
		//Example : Combine flag30 and flag31
		//unsigned flag29     :1;
		//unsigned flag30     :2;
		//the value of sys.flag30 range from 0 to 3 then overflow to 0 again and vice versa
		//the value of flag29 is not affected when flag30 overflow
	};
}sys_t;

union{
	uint16_t flags;
	struct{
		unsigned path       :3;
		unsigned task       :3;
		unsigned cyon	  	:1;
		unsigned laser	    :1;
		unsigned base	    :3;
		unsigned baseok	    :1;
		unsigned baseleft   :1;
	};
}pp;

struct{
	int svbH1;
	int svbH2;
	int svbV1;
	int svbV2;
	int svlgf1;
	int svlgf2;
	int svlgf3;
	int svlgf21;
	int svlgf22;
	int svlgf23;
	union{
		uint16_t flags;
		struct{
			uint16_t svapp		:1;
			uint16_t cyapp		:1;
			uint16_t flag4		:1;
			uint16_t flag5		:1;
			uint16_t flag6		:1;
			uint16_t flag7		:1;
			uint16_t flag8		:1;
			uint16_t flag9		:1;
			uint16_t flag10		:1;
			uint16_t flag11		:1;
			uint16_t flag12		:1;
			uint16_t flag13		:1;
			uint16_t flag14		:1;
			uint16_t flag15		:1;
		};
	};
}fx;

union{
	uint16_t flags;
	struct{
		uint16_t bldclapp			:1;
		uint16_t bldcfapp			:1;
		uint16_t bldclayer			:3;
		uint16_t bldcflayer			:3;
		uint16_t flip				:2;
		uint16_t clamp				:1;
		uint16_t release			:1;
		uint16_t pwstate			:2;
		uint16_t gripok				:1;
		uint16_t bldcstart			:1;	//16
	};
}lagori;

sys_t sys;

void RNS_config(CAN_HandleTypeDef* hcanx);
void set(void);
void seeker_mode(void);
void lagori_func(void);
void lifting_func(void);

void hitter_mode(void);
void pp_runpath(void);
void pp_func(void);

void setPP_points(void);

void button(void);
void limit_sw(void);
void analog_sr(void);
void functest(void);
void enq(void);
void SERVO_getPulse(void);
void KFilterAdjust(void);

void Await(uint32_t ticks);
#define NUM_INT_UPDATE		1
#define NUM_FLOAT_UPDATE	1

void ILI9341_Init_List(void);
void ILI9341_Update_List(void);
#endif /* INC_COMMON_H_ */
