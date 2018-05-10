#ifndef _FASTMATH_H
#define _FASTMATH_H

#include "stdint.h"

#define M_PI ((double)3.14159265358979323846)

int32_t fastSin(int32_t angle);
int32_t fastCos(int32_t angle);

double calcPhaseDouble(uint8_t* DataFile_1, uint8_t* DataFile_2, uint32_t N, double Freq, double TDiscret);

int32_t integrate(uint8_t* data1, uint8_t* data2, uint32_t length);


#endif // _FASTMATH_H
