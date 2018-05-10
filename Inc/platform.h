#ifndef _ABSTRACT_LAYER
#define _ABSTRACT_LAYER

#include "main.h"

#define SETPIN(port, pin, value)	if(value) LL_GPIO_SetOutputPin(port, pin); else LL_GPIO_ResetOutputPin(port, pin);

#define DevicePower(a)				SETPIN(SupplyEnable_GPIO_Port, SupplyEnable_Pin, a)
#define Backlight(a)				SETPIN(Backlignt_GPIO_Port, Backlignt_Pin, !a)
#define LaserEnable(a)				SETPIN(LaserEnable_GPIO_Port, LaserEnable_Pin, !a)
#define LaserSelect(a)				SETPIN(LaserChange_GPIO_Port, LaserChange_Pin, a)
#define LASER_INTERNAL 				0
#define LASER_EXTERNAL 				1
#define LaserPower(a)				SETPIN(LaserEnable2_GPIO_Port, LaserEnable2_Pin, a)
#define LASER_POWER_LOW				0
#define LASER_POWER_HIGH 			1
#define ButtonRead()				LL_GPIO_IsInputPinSet(Button_GPIO_Port, Button_Pin)
#define ModulationEnable(a) 		SETPIN(Modulation_enable_GPIO_Port, Modulation_enable_Pin, a)
#define SdaPinWrite(a)				SETPIN(SDA_GPIO_Port, SDA_Pin, a)
#define SdaPinRead()				LL_GPIO_IsInputPinSet(SDA_GPIO_Port, SDA_Pin)
#define SclPinWrite(a)				SETPIN(SCL_GPIO_Port, SCL_Pin, a)
#define HVPwmSetDuty(a)		  		TIM3->CCR1 = (uint16_t)a

#define GenVoltage()				LL_GPIO_IsInputPinSet(CKLIN_GPIO_Port, CKLIN_Pin)

typedef enum 
{
	FREQ_50M_DFREQ_10K,
	FREQ_50M_DFREQ_100K
} PLL_Freq_t;

void ADCDMATIM_Prepare(void);
void ADCDMATIM_Start(void);
void ADCDMATIM_Stop(void);
void CaptureSignals(uint8_t* arrADC, uint8_t* arrGEN, int length);
void WTFSignal(void);

void ADC_Calibrate(void);
uint32_t ADCGetAmplitude(void);
void AdjustHighVoltage(void);

void InitPLL(PLL_Freq_t mod);

void ParseUartCmd(void);

void PlatformInit(void);
void PlatformLoop(void);

float CalcPhaseFourier(void);


#endif // _ABSTRACT_LAYER
