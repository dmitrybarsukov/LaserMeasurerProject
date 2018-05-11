#include "platform.h"
#include "si5351drv.h"
#include "stdint.h"
#include "delay.h"
#include "stdio.h"
#include "fastmath.h"
#include "math.h"

int continousRun = 0;
int pllmod = 1;
int avg = 1;
int HSI14calibValue = 0x10;
int ADC_capture_size = 1000;


extern volatile int INTcount;
extern volatile int INTflagEnd;
extern volatile int INTsamplesCount;
extern volatile int INTflagStart;

float global_offset;

void PlatformInit(void)
{	
	DevicePower(1);
	printf("Loading...\r\n");
	DelayMs(100);
	
	printf("Calibrating and starting ADC at 1 MHz\r\n");
	ADC_Calibrate();
	
	printf("Starting PLL\r\n");
	InitPLL(FREQ_50M_DFREQ_10K);
	
	printf("Enabling modulation\r\n");
	ModulationEnable(1);
	
	printf("Configuring lasers\r\n");
	LaserPower(LASER_POWER_LOW);
	LaserEnable(1);
	LaserSelect(LASER_INTERNAL);

	DelayMs(10);
	printf("Adjusting PWM 1\r\n");
	AdjustHighVoltage();
	
	DelayMs(10);
	float angle = 0;
	for(int i = 0; i < 10; i++)
	{
		angle += CalcPhaseFourier() / 10;
	}
	global_offset = angle;
	
	printf("Global offset is %.2f\r\n", global_offset);
	
	LaserSelect(LASER_EXTERNAL);
	DelayMs(10);
	printf("Adjusting PWM 2\r\n");
	AdjustHighVoltage();
	
	//printf("Setting PWM to 16%%\r\n");
	//HVPwmSetDuty(114);
	
	//printf("0 0\r\n0 360\r\n");
	
	DelayMs(500);

//	WTFSignal();
	
	

	return;
	
}


void PlatformLoop(void)
{
	if(continousRun)
	{
		float angle = 0;
		for(int i = 0; i < avg; i++)
		{
			LaserSelect(LASER_EXTERNAL);
			angle += CalcPhaseFourier() / avg;
			//angle -= global_offset;
			if(angle < 0)
				angle += 360;
		}
		printf("%.2f\r\n", angle);
	}

	ParseUartCmd();
}


void InitPLL(PLL_Freq_t mod)
{
	NVIC_DisableIRQ(EXTI4_15_IRQn);
	Si5351_ResetPLL();
	
	Si5351_SetLoadCap(SI5351_XTAL_LOAD_CAP_10pf);
	Si5351_SetupPllSrc(SI5351_PLLxSRC_DIV1 |
					   SI5351_PLLASRC_XTAL |
					   SI5351_PLLBSRC_XTAL);
		
	Si5351_EnableOutputs(0x00);
// 10 MHz, 10 kHz
	if(mod == FREQ_50M_DFREQ_10K)
	{
		Si5351_ClkxConfig(0, SI5351_CLKx_ON |
							 SI5351_CLKx_MSx_Integer |
							 SI5351_CLKx_MSx_SRC_PLLA |
							 SI5351_CLKx_NOINVERT | 
							 SI5351_CLKx_SRC_MSx |
							 SI5351_CLKx_IDRV_2ma);
		
		Si5351_ClkxConfig(1, SI5351_CLKx_ON |
							 SI5351_CLKx_MSx_Integer |
							 SI5351_CLKx_MSx_SRC_PLLA |
							 SI5351_CLKx_NOINVERT | 
							 SI5351_CLKx_SRC_MSx |
							 SI5351_CLKx_IDRV_8ma);
		
		Si5351_ClkxConfig(2, SI5351_CLKx_ON |
							 SI5351_CLKx_MSx_Integer |
							 SI5351_CLKx_MSx_SRC_PLLB |
							 SI5351_CLKx_NOINVERT | 
							 SI5351_CLKx_SRC_MSx |
							 SI5351_CLKx_IDRV_2ma);
											 
		Si5351_PllMSnRawConfig(0, 3280, 16, 27);
		Si5351_PllMSnRawConfig(1, 3279, 563, 675);
		
		Si5351_MSnRawConfig(0, 1536, 0, 1, SI5351_MSx_DIV1);
		Si5351_MSnRawConfig(1, 79488, 0, 1, SI5351_MSx_DIV128);
		Si5351_MSnRawConfig(2, 1536, 0, 1, SI5351_MSx_DIV1);
	}
	else if(mod == FREQ_50M_DFREQ_100K)
	{
// 50 MHz, 100 kHz
		
		Si5351_ClkxConfig(0, SI5351_CLKx_ON |
							 SI5351_CLKx_MSx_Integer |
							 SI5351_CLKx_MSx_SRC_PLLA |
							 SI5351_CLKx_NOINVERT | 
							 SI5351_CLKx_SRC_MSx |
							 SI5351_CLKx_IDRV_2ma);
		
		Si5351_ClkxConfig(1, SI5351_CLKx_ON |
							 SI5351_CLKx_MSx_Integer |
							 SI5351_CLKx_MSx_SRC_PLLA |
							 SI5351_CLKx_NOINVERT | 
							 SI5351_CLKx_SRC_MSx |
							 SI5351_CLKx_IDRV_8ma);
		
		Si5351_ClkxConfig(2, SI5351_CLKx_ON |
							 SI5351_CLKx_MSx_Integer |
							 SI5351_CLKx_MSx_SRC_PLLB |
							 SI5351_CLKx_NOINVERT | 
							 SI5351_CLKx_SRC_MSx |
							 SI5351_CLKx_IDRV_2ma);
											 
		Si5351_PllMSnRawConfig(0, 3280, 16, 27);
		Si5351_PllMSnRawConfig(1, 3273, 1, 135);
		
		Si5351_MSnRawConfig(0, 1536, 0, 1, SI5351_MSx_DIV1);
		Si5351_MSnRawConfig(1, 25088, 0, 1, SI5351_MSx_DIV1);
		Si5351_MSnRawConfig(2, 1536, 0, 1, SI5351_MSx_DIV1);
	}
	
	Si5351_ResetPLL();
	Si5351_EnableOutputs(0x07);
}

void ParseUartCmd(void)
{
	char ch;
	if(scanf("%c", &ch) != EOF)
	{
		switch(ch)
		{
		case '+': 
			TIM3->CCR1++; 
			DelayMs(100);
			printf("PWM is now %.2f %% %d\r\n", TIM3->CCR1 / 6.0, ADCGetAmplitude());
			break;
		case '-':
			TIM3->CCR1--;
			DelayMs(100);
			printf("PWM is now %.2f %% %d\r\n", TIM3->CCR1 / 6.0, ADCGetAmplitude());
			break;
		case '*':
			AdjustHighVoltage();
			break;
		case '/':
			printf("ADC amplitude %d\r\n", ADCGetAmplitude());
			break;
		case 'e':
			LaserSelect(LASER_EXTERNAL);
			break;
		case 'i':
			LaserSelect(LASER_INTERNAL);
			break;
		case 'r':
			continousRun = !continousRun;
			break;
		case '1':
			avg++;
			printf("AVG = %d\r\n", avg);
			break;
		case '2':
			if(avg > 1)
				avg--;
			printf("AVG = %d\r\n", avg);
			break;
		case '3':
			HSI14calibValue++;
			LL_RCC_HSI14_SetCalibTrimming(HSI14calibValue);
			printf("Calibration value = %d      ", HSI14calibValue);
			DelayUs(1000);
			testCaptureData();
			break;
		case '4':
			HSI14calibValue--;	
			LL_RCC_HSI14_SetCalibTrimming(HSI14calibValue);
			printf("Calibration value = %d      ", HSI14calibValue);
			DelayUs(1000);
			testCaptureData();
			break;
		case '5':
			ADC_capture_size += 100;
			printf("%d", ADC_capture_size);
			break;
		case '6':
			ADC_capture_size -= 100;
			printf("%d", ADC_capture_size);
			break;
		default:
			printf("Err %c\r\n", ch);
			break;
		}
	}
}

uint32_t ADCGetAmplitude(void)
{
	const int CAPTURES_PER_TEST = 500;
	uint8_t dat[CAPTURES_PER_TEST];
	int min = 100500, max = 0;
	
	CaptureSignalDMA(dat, CAPTURES_PER_TEST, WAIT);
	
	for(int i = 0; i < CAPTURES_PER_TEST; i++)
	{
		if(dat[i] < min)
			min = dat[i];
		if(dat[i] > max)
			max = dat[i];
	}
	return max - min;
}

void AdjustHighVoltage(void)
{
	const int PWM_DUTY_MIN = 70;
	const int PWM_DUTY_MAX = 100;
	const int PWM_DUTY_DELTA = (PWM_DUTY_MAX - PWM_DUTY_MIN);

	int ampls[PWM_DUTY_DELTA];
	int currentDuty;
	int i;
	for(i = 0; i < PWM_DUTY_DELTA; i++)
	{
		currentDuty = PWM_DUTY_MIN + i;
		HVPwmSetDuty(currentDuty);
		DelayMs(100);
		ampls[i] = ADCGetAmplitude();
		printf("%d\r\n", ampls[i]);
	}
	for(i = 1; i < PWM_DUTY_DELTA; i++)
	{
		if(ampls[i] > 250)
			break;
	}
	i -= 2;
	HVPwmSetDuty(PWM_DUTY_MIN + i);
	printf("PWM is %.2f %% %d\r\n", TIM3->CCR1 / 6.0, ampls[i]);
}

void ADC_Calibrate(void)
{
	LL_ADC_Disable(ADC1);
	DelayUs(10);
	LL_ADC_StartCalibration(ADC1);
	DelayUs(10);
	while(LL_ADC_IsCalibrationOnGoing(ADC1));
	LL_ADC_Enable(ADC1);
	
}

void WTFSignal(void)
{
	const int DATALEN = 2000;
	uint8_t adcdata[DATALEN];
	
	CaptureSignalDMA(adcdata, DATALEN, WAIT);
	
	for(int i = 0; i < DATALEN; i++)
		printf("%d\r\n", adcdata[i]);
	
}

void ADCDMA_Prepare(uint8_t* address, int size)
{
	LL_ADC_REG_StopConversion(ADC1);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)address);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&(ADC1->DR) + 1);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, size);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	INTflagStart = 1;
	INTflagEnd = 0;
	INTsamplesCount = 0;
}

void ADCDMA_Start(void)
{
	//LL_ADC_REG_StartConversion(ADC1);
	//while(GenVoltage());
	while(!GenVoltage());
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
	NVIC_EnableIRQ(EXTI4_15_IRQn);	
	//LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_9);
}

void ADCDMA_Stop(void)
{
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_ADC_REG_StopConversion(ADC1);
}

void CaptureSignalDMA(uint8_t* arrADC, int length, int wait)
{
	ADCDMA_Prepare(arrADC, length);
	INTcount = length / 100;
	//while(GenVoltage());
	//while(!GenVoltage());
	ADCDMA_Start();
	if(wait)
		while(!INTflagEnd);
	else
		DelayUs(50);
	//while(!LL_DMA_IsActiveFlag_TC1(DMA1));
	

	
	//ADCDMA_Stop();
}


float CalcPhaseFourier(void)
{	
	//const int DATALEN = 1000;
	int DATALEN = ADC_capture_size;
	uint8_t adcdata[2050];
	
	CaptureSignalDMA(adcdata, DATALEN, WAIT);
/*
	int Re =  integrateFunc(adcdata, fastSin, DATALEN);
	int Im = -integrateFunc(adcdata, fastCos, DATALEN);
*/
	int Re =  integrateAutoSin(adcdata, DATALEN);
	int Im = -integrateAutoCos(adcdata, DATALEN);
	
	float angle;
	
	if(Re >= 0 && Im >= 0)
		angle =       atanf((float)Im / Re) / M_PI * 180;
	else if(Re < 0 && Im >= 0)
		angle = 90  - atanf((float)Re / Im) / M_PI * 180;
	else if(Re < 0 && Im < 0)
		angle = 180 + atanf((float)Im / Re) / M_PI * 180;
	else
		angle = 270 - atanf((float)Re / Im) / M_PI * 180;
	
	//printf("%.2f\r\n", angle);
	return angle;
}

void testCaptureData(void)
{
	const int ARRSIZE = 2100;
	uint8_t data[ARRSIZE];
	ADCDMA_Prepare(data, ARRSIZE);
	INTcount = 20;
	//while(GenVoltage());
	//while(!GenVoltage());
//	printf("Capturing %d Periods\r\n", INTcount);
	ADCDMA_Start();
	//while(!LL_DMA_IsActiveFlag_TC1(DMA1));
	
	while(!INTflagEnd);
	printf("Captured %d samples\r\n", ARRSIZE - INTsamplesCount);
/*	
	for(int i = 0; i < ARRSIZE - INTsamplesCount; i++)
		printf("%d\r\n", data[i]);
*/
}
