#include "fastmath.h"
#include "stdint.h"

#include "math.h"

const int16_t quarterPeriod[26] = { 0,
	48	,	95	,	142	,	189	,	236	,
	282	,	327	,	372	,	415	,	458	,
	500	,	541	,	580	,	618	,	655	,
	690	,	724	,	756	,	786	,	815	,
	841	,	866	,	889	,	910	,	928	,
	945	,	959	,	972	,	982	,	990	,	995	,	999
};

int32_t fastSin(int32_t angle) {
	int sign = 1;
	while(angle < 0)
		angle += 10000;
	angle = angle % 100;

	if(angle > 75) {
		sign = -1;
		angle = 100 - angle;
	} else if(angle > 50) {
		sign = -1;
		angle -= 50;
	} else if(angle > 25)
		angle = 50 - angle;
	return sign * quarterPeriod[angle];
}

int32_t fastCos(int32_t angle) {
	return fastSin(angle + 32);
}

double calcPhaseDouble(uint8_t* DataFile_1, uint8_t* DataFile_2, uint32_t N, double Freq, double TDiscret)
{
  const uint32_t m = (N % 2) ? (N - 1) : N;
        uint32_t k;  
  
  const double StepFi = 2 * M_PI * Freq * TDiscret;
        double fi, cosx, sinx;
        double I_cos_1 = 0, I_sin_1 = 0;
        double I_cos_2 = 0, I_sin_2 = 0;
        
  double  dUnit_1;
  double  dUnit_2;
  
  for(k = m - 2; k; k -= 2)
  {    
    fi       = StepFi * k;
    cosx     = cos(fi);
    sinx     = sin(fi);
    dUnit_1  = (double)DataFile_1[k];
    I_cos_1 += dUnit_1 * cosx;
    I_sin_1 += dUnit_1 * sinx;
    dUnit_2  = (double)DataFile_2[k];
    I_cos_2 += dUnit_2 * cosx;
    I_sin_2 += dUnit_2 * sinx;
  }
  
  I_cos_1  /= 2;
  I_sin_1  /= 2;
  I_cos_2  /= 2;
  I_sin_2  /= 2;

  for (k = m - 1; ~k; k -= 2)
  {    
    fi       = StepFi * k;
    cosx     = cos(fi);
    sinx     = sin(fi);
    dUnit_1  = (double)DataFile_1[k];
    I_cos_1 += dUnit_1 * cosx;
    I_sin_1 += dUnit_1 * sinx;
    dUnit_2  = (double)DataFile_2[k];
    I_cos_2 += dUnit_2 * cosx;
    I_sin_2 += dUnit_2 * sinx;
  }

  I_cos_1 *= 4;
  I_sin_1 *= 4;
  I_cos_2 *= 4;
  I_sin_2 *= 4;
   
  fi       = StepFi * m;
  cosx     = cos(fi);
  sinx     = sin(fi);
  dUnit_1  = ((double) DataFile_1[m]); 
  I_cos_1 += dUnit_1 * cosx + DataFile_1[0];
  I_sin_1 += dUnit_1 * sinx;
  dUnit_2  = ((double) DataFile_2[m]);
  I_cos_2 += dUnit_2 * cosx + DataFile_2[0];
  I_sin_2 += dUnit_2 * sinx;

  double angle1 = atan((double)I_sin_1 / I_cos_1);
  if(I_sin_1 < 0)
	  angle1 += M_PI;
  
  double angle2 = atan((double)I_sin_2 / I_cos_2);
  if(I_sin_2 < 0)
	  angle2 += M_PI;
  
  return angle2 - angle1;
}

int32_t integrate(uint8_t* data1, uint8_t* data2, uint32_t length)
{
	int mean1 = 127, mean2 = 127;
	
	int32_t sum = 0;
	for(int i = 0; i < length; i++)
		sum += ((int)data1[i] - mean1) * ((int)data2[i] - mean2);

	return sum;
}






