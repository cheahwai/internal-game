/*
 * Tune.h
 *
 *  Created on: Mar 18, 2022
 *      Author: lokcharming
 */

#ifndef SRC_TUNE_H_
#define SRC_TUNE_H_

//num_int_tune + num_float_tune in a page must less than 72
#define NUM_INT_TUNE_LIST0			0
#define NUM_FLOAT_TUNE_LIST0		12

#define NUM_INT_TUNE_LIST1			0
#define NUM_FLOAT_TUNE_LIST1		70

#define NUM_INT_TUNE_LIST2			0
#define NUM_FLOAT_TUNE_LIST2		16

#define NUM_INT_TUNE_LIST3			1
#define NUM_FLOAT_TUNE_LIST3		1

#define NUM_INT_TUNE_LIST4			1
#define NUM_FLOAT_TUNE_LIST4		1

float AP, AI, AD, BP, BI, BD, CP, CI, CD, DP, DI, DD;

float point1[7];
float point2[7];
float point3[7];
float point4[7];
float point5[7];
float point6[7];
float point7[7];
float point8[7];
float point9[7];
float point10[7];
float point11[7];
float point12[7];
float point13[7];
float point14[7];

float pLNE[7];
float pLSE[7];
float pLNW[7];
float pLSW[7];

void registerPath(float path[][7], int numPoint, ...);

#endif /* SRC_TUNE_H_ */
