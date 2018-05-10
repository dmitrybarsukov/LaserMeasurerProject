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

float global_offset;

void PlatformInit(void)
{	
	DevicePower(1);
	printf("Loading...\r\n");
	DelayMs(5000);
	
	printf("Calibrating and starting ADC at 1 MHz\r\n");
	ADC_Calibrate();
	LL_ADC_REG_StartConversion(ADC1);
	
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
	//HVPwmSetDuty(75);
	AdjustHighVoltage();
	
	DelayMs(10);
	float angle = 0;
	for(int i = 0; i < 1000; i++)
	{
		angle += CalcPhaseFourier() / 1000;
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
	
	DelayMs(100);

	WTFSignal();
	
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
			angle -= global_offset;
			if(angle < 0)
				angle += 360;
		}
		printf("%.2f\r\n", angle);
	}

	ParseUartCmd();
}


void InitPLL(PLL_Freq_t mod)
{
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
		default:
			printf("Err %c\r\n", ch);
			break;
		}
	}
}

uint32_t ADCGetAmplitude(void)
{
	const int CAPTURES_PER_TEST = 1000;
	uint8_t dat[CAPTURES_PER_TEST];
	int min = 100500, max = 0;
	
	CaptureSignals(dat, NULL, CAPTURES_PER_TEST);
	
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
	const int PWM_DUTY_MIN = 60;
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
	const int DATALEN = 1000;
	uint8_t adcdata[DATALEN];
	uint8_t gendata[DATALEN];
	
	CaptureSignals(adcdata, gendata, DATALEN);
	
	for(int i = 0; i < 1000; i++)
		printf("%d %d\r\n", adcdata[i], gendata[i]);
	
}

void ADCDMATIM_Prepare(uint8_t* address, int size)
{
	// Setup DMA
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)address);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&(ADC1->DR));
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, size);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	
	// Setup TIM
	LL_TIM_SetCounter(TIM1, 0);
}

void ADCDMATIM_Start(void)
{
	LL_ADC_REG_StartConversion(ADC1);
	LL_TIM_EnableCounter(TIM1);
}

void ADCDMATIM_Stop(void)
{
	LL_TIM_DisableCounter(TIM1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_ADC_REG_StopConversion(ADC1);

}

void CaptureSignals(uint8_t* arrADC, uint8_t* arrGEN, int length)
{
	ADCDMATIM_Prepare(arrADC, length);
	
	while(!GenVoltage());
	while(GenVoltage());
	ADCDMATIM_Start();
	
	while(!LL_DMA_IsActiveFlag_TC1(DMA1));
	
	ADCDMATIM_Stop();
	
}


float CalcPhaseFourier(void)
{	
	const int DATALEN = 1000;
	uint8_t adcdata[DATALEN];
	uint8_t gendata[DATALEN];
	
	CaptureSignals(adcdata, gendata, DATALEN);
		
	int Re = integrate(gendata, adcdata + 0 , DATALEN - 100);
	int Im = integrate(gendata, adcdata + 25, DATALEN - 100);
	
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
