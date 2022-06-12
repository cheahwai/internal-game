/*
 * Tune.c
 *
 *  Created on: Mar 18, 2022
 *      Author: lokcharming
 */

#include "TuningInterface/TuningInterface.h"
#include "Tune.h"

void registerPath(float path[][7], int numPoint, ...){
	va_list pinconfig;
	va_start(pinconfig, numPoint);
	float* point;
	for(int i=0; i<numPoint; i++){
		point = va_arg(pinconfig, float *);
		for(int j=0; j<7; j++){
			path[i][j] = point[j];
		}
	}
	va_end(pinconfig);
}

Tune_Int_t TuneIntList0[NUM_INT_TUNE_LIST0] = {

	//varI()
};

Tune_Float_t TuneFloatList0[NUM_FLOAT_TUNE_LIST0]={
		varF(AP, 0.500, 5.0),
		varF(AI, 0.500, 5.0),
		varF(AD, 0.000, 0.5),
		varF(BP, 0.500, 5.0),
		varF(BI, 0.500, 5.0),
		varF(BD, 0.000, 0.5),
		varF(CP, 0.500, 5.0),
		varF(CI, 0.500, 5.0),
		varF(CD, 0.000, 0.5),
		varF(DP, 0.500, 5.0),
		varF(DI, 0.500, 5.0),
		varF(DD, 0.000, 0.5),
};

Tune_Int_t TuneIntList1[NUM_INT_TUNE_LIST1] = {

};

/*
* 		[][0]= minimum speed
* 		[][1]= x-coordinate
* 		[][2]= y-coordinate
* 		[][3]= z-coordinate
* 		[][4]= xy pid output
* 		[][5]= Point Lock
* 		[][6]= Curve Control Radius
*/
Tune_Float_t TuneFloatList1[NUM_FLOAT_TUNE_LIST1]={
		varF(point1[0], 0.0, 10.0),
		varF(point1[1], 0.0, 6.0),
		varF(point1[2], 0.0, 3.0),
		varF(point1[3], 0.0, 90.0),
		varF(point1[4], 0.0, 5.0),

		varF(point2[0], 0.0, 10.0),
		varF(point2[1], 0.0, 6.0),
		varF(point2[2], 0.0, 6.0),
		varF(point2[3], 0.0, 90.0),
		varF(point2[4], 0.0, 5.0),

		varF(point3[0], 0.0, 10.0),
		varF(point3[1], 0.0, 6.0),
		varF(point3[2], 0.0, 6.0),
		varF(point3[3], 0.0, 90.0),
		varF(point3[4], 0.0, 5.0),

		varF(point4[0], 0.0, 10.0),
		varF(point4[1], 0.0, 6.0),
		varF(point4[2], 0.0, 6.0),
		varF(point4[3], 0.0, 90.0),
		varF(point4[4], 0.0, 5.0),

		varF(point5[0], 0.0, 10.0),
		varF(point5[1], 0.0, 6.0),
		varF(point5[2], 0.0, 6.0),
		varF(point5[3], 0.0, 90.0),
		varF(point5[4], 0.0, 5.0),

		varF(point6[0], 0.0, 10.0),
		varF(point6[1], 0.0, 6.0),
		varF(point6[2], 0.0, 2.0),
		varF(point6[3], 0.0, 90.0),
		varF(point6[4], 0.0, 5.0),

		varF(point7[0], 0.0, 10.0),
		varF(point7[1], -1.0, 3.0),
		varF(point7[2], 0.0, 2.0),
		varF(point7[3], 0.0, 90.0),
		varF(point7[4], 0.0, 5.0),

		varF(point8[0], 0.0, 10.0),
		varF(point8[1], -1.0, 1.0),
		varF(point8[2], 0.0, 2.0),
		varF(point8[3], 0.0, 90.0),
		varF(point8[4], 0.0, 5.0),

		varF(point9[0], 0.0, 10.0),
		varF(point9[1], -6.0, 0.0),
		varF(point9[2], 0.0, 6.0),
		varF(point9[3], -90.0, 0.0),
		varF(point9[4], 0.0, 5.0),

		varF(point10[0], 0.0, 10.0),
		varF(point10[1], -6.0, 0.0),
		varF(point10[2], 0.0, 6.0),
		varF(point10[3], -90.0, 0.0),
		varF(point10[4], 0.0, 5.0),

		varF(point11[0], 0.0, 10.0),
		varF(point11[1], -6.0, 0.0),
		varF(point11[2], 0.0, 6.0),
		varF(point11[3], -90.0, 0.0),
		varF(point11[4], 0.0, 5.0),

		varF(point12[0], 0.0, 10.0),
		varF(point12[1], -6.0, 0.0),
		varF(point12[2], 0.0, 6.0),
		varF(point12[3], -90.0, 0.0),
		varF(point12[4], 0.0, 5.0),

		varF(point13[0], 0.0, 10.0),
		varF(point13[1], -6.0, 3.0),
		varF(point13[2], 0.0, 2.0),
		varF(point13[3], -90.0, 0.0),
		varF(point13[4], 0.0, 5.0),

		varF(point14[0], 0.0, 10.0),
		varF(point14[1], -1.0, 1.0),
		varF(point14[2], 0.0, 2.0),
		varF(point14[3], -90.0, 0.0),
		varF(point14[4], 0.0, 5.0),

};

Tune_Int_t TuneIntList2[NUM_INT_TUNE_LIST2] = {

};

Tune_Float_t TuneFloatList2[NUM_FLOAT_TUNE_LIST2]={

		varF(pLNE[0], 0.0, 10.0),
		varF(pLNE[1], 0.0, 2.0),
		varF(pLNE[2], 0.0, 8.0),
		varF(pLNE[4], 0.0, 10.0),

		varF(pLNW[0], 0.0, 10.0),
		varF(pLNW[1], -2.0, 0.0),
		varF(pLNW[2], 0.0, 8.0),
		varF(pLNW[4], 0.0, 10.0),

		varF(pLSE[0], 0.0, 10.0),
		varF(pLSE[1], 0.0, 2.0),
		varF(pLSE[2], 0.0, 6.0),
		varF(pLSE[4], 0.0, 10.0),

		varF(pLSW[0], 0.0, 10.0),
		varF(pLSW[1], -2.0, 0.0),
		varF(pLSW[2], 0.0, 6.0),
		varF(pLSW[4], 0.0, 10.0),
};

Tune_Int_t TuneIntList3[NUM_INT_TUNE_LIST3] = {

};

Tune_Float_t TuneFloatList3[NUM_FLOAT_TUNE_LIST3]={

};

Tune_Int_t TuneIntList4[NUM_INT_TUNE_LIST4] = {

};

Tune_Float_t TuneFloatList4[NUM_FLOAT_TUNE_LIST4]={

};
