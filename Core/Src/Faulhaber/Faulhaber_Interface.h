
/*******************************************************************************
 * Title   : Laser (
 * Author  : LokCharming
 * Version : 1.00
 * Date    : Sept 2022
 *******************************************************************************
 * Description:
 * - Combined with ADC and kalman_filter(KF) to fully used it
 * -
 *
 * Version History:
 * 1.00 by Klok
 * - Basic function of laser calibration
 *
 * 1.1 by Anas
 * - Added Check for distance and way to manually tune
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef SRC_FAULHABER_INTERFACE_FAULHABER_INTERFACE_H_
#define SRC_FAULHABER_INTERFACE_FAULHABER_INTERFACE_H_

#include "../BIOS/bios.h"
#include "../CAN/can.h"

/**************************************************
 * 		Structure							  	  *
 *************************************************/
struct{

}FAULHABER_INTERFACE_t;






/**************************************************
 * 		Enumerator							  	  *
 *************************************************/
typedef enum{
	FAULHABER1 = 78,
	FAULHABER2 = 79,
	FAULHABER3 = 80,
	FAULHABER4 = 81,
	FAULHABER5 = 82,
	FAULHABER6 = 83,
	FAULHABER7 = 84,
	FAULHABER8 = 85,
}MC5010S_CO_NODE_ID;

typedef enum{
	NMT 				= 0b0000,
	SYNC 				= 0b0001,
	EMERGENCY			= 0b0001,
	PDO1_TX 			= 0b0011,
	PDO1_RX 			= 0b0100,
	PD02_TX				= 0b0101,
	PDO2_RX				= 0b0110,
	PDO3_TX				= 0b0111,
	PD03_RX				= 0b1000,
	PD04_TX				= 0b1001,
	PD04_RX				= 0b1010,
	SDO_TX				= 0b1011,
	SDO_RX				= 0b1100,
	NMT_ERROR_CONTROL	= 0b1110
}FUNCTIONAL_CODE;
/**************************************************
 * 		Extern	variables					  	  *
 *************************************************/


/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/
void FaulWriteOD(MC5010S_CO_NODE_ID nodeId);
void FaulReadOD(MC5010S_CO_NODE_ID nodeId);







#endif /* SRC_FAULHABER_INTERFACE_FAULHABER_INTERFACE_H_*/
