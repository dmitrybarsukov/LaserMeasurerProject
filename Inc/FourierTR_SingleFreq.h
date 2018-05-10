/**
  ******************************************************************************
  * @file    SoftMCU/SPIDevice/FourierTR_SingleFreq.h 
  * @author  Plotnikov S.A.
  * @version V1.0.0
  * @date    09-December-2013
  * @brief   This file contains the prototype of Fourier transform function
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FOURIERTR_SINGLEFREQ_H
#define __FOURIERTR_SINGLEFREQ_H

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

template <typename T> std::complex <double> CalculateImpedance 
       (T *, T *, uint32_t, double, double);


//float getPhase(int16_t* data1, int16_t* data2, uint32_t size, uint32_t period);

#endif /* __FOURIERTR_SINGLEFREQ_H */

/*** (C) COPYRIGHT 2013 Electronic Devices Department of NSTU ***END OF FILE***/
