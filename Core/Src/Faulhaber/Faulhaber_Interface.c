/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "Faulhaber_Interface.h"
/*********************************************/

/********* DEVELOPING *********/


/*********************************************/
/*          Variable                         */
/*********************************************/

/*********************************************/

/*********************************************/
/*           Subroutine Function             */
/*********************************************/

/*
 * Function Name		: laser_init
 * Function Description : This function is called to calibrate laser_sensor.
 * Function Remarks		: User need to initialize ADC and KF first before calling this function. To Tune Max Distance use 1 meter ruler to check accuracy.
 * Function Arguments	: min_value			minimum value of ADC/KF return (float for KF)
 * 						  max_value			maximum value of ADC/KF return (float for KF)
 * 						  min_distance 		minimum distance,in meter when min_value detected (can take from encoder)
 * 						  max_distance		maximum distance,in meter when max_value detected (can take from encoder)
 * 						  input				pointer to input of laser (can be either from ADC or KF )
 * 						  output			pointer to output of laser
 * 						  laser				pointer to structure LASER_t
 * Function Return		: None
 * Function Example		: LaserInit(121, 4035, 0.46, 2.4, &L_laser, &L_distance, &Left_laser);
 */




