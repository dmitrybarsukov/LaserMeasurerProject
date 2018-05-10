/**
  ******************************************************************************
  * @file    SoftMCU/SPIDevice/FourierTR_SingleFreq.cpp
  * @author  Plotnikov S.A.
  * @version V1.0.0
  * @date    13-December-2013
  * @brief   This file contains function Fourier transform and 
             service functions.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <complex>
#include <stdint.h>

#include "FourierTR_SingleFreq.h"

template std::complex <double> CalculateImpedance <int8_t>
       (int8_t*, int8_t*, uint32_t, double, double);

template std::complex <double> CalculateImpedance <int16_t>
       (int16_t*, int16_t*, uint32_t, double, double);

template std::complex <double> CalculateImpedance <int32_t>
       (int32_t*, int32_t*, uint32_t, double, double);

template std::complex <double> CalculateImpedance <int64_t>
       (int64_t*, int64_t*, uint32_t, double, double);

template std::complex <double> CalculateImpedance <float>
       (float*, float*, uint32_t, double, double);

template std::complex <double> CalculateImpedance <double>
       (double*, double*, uint32_t, double, double);

#ifndef M_PI
  #define M_PI            ((double)3.14159265358979323846)
#endif

/* Private variables ---------------------------------------------------------*/
/* Private functions -------------------------------------------------------- */

/*******************************************************************************
* Function Name  : CalculateImpedance
* Description    : This function calculates impedance.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
template <typename T> std::complex <double> CalculateImpedance 
       (T *DataFile_1, T *DataFile_2, uint32_t N, double Freq, double TDiscret)
{
  const uint32_t m = (N % 2) ? (N - 1) : N;
        uint32_t k;  
  
  const double StepFi = 2 * M_PI * Freq * TDiscret;
        double fi, cosx, sinx;
        double I_cos_1 = 0, I_sin_1 = 0;
        double I_cos_2 = 0, I_sin_2 = 0;
        
  double  dUnit_1;
  double &dUnit_2 = dUnit_1;
  
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

  return std::complex <double> (I_cos_1, I_sin_1) / 
         std::complex <double> (I_cos_2, I_sin_2); 
}

/*** (C) COPYRIGHT 2013 Electronic Devices Department of NSTU ***END OF FILE***/
