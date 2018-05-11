#include "delay.h"
#include "stm32f0xx_ll_system.h"
#include "stdint.h"

extern volatile uint32_t SysTimeSeconds;

void DelayMs(uint32_t t)
{
	t = t * 1000 + SysTimeUs();
	while(SysTimeUs() < t);
	return;
}

void DelayUs(uint32_t t)
{
	t += SysTimeUs();
	while(SysTimeUs() < t);
	return;
}

uint32_t SysTimeS()
{
	return SysTimeSeconds;
}

uint32_t SysTimeMs()
{
	return SysTimeSeconds * 1000 + (SysTick->LOAD - SysTick->VAL) / 6000;
}

uint32_t SysTimeUs()
{
	return SysTimeSeconds * 1000000 + (SysTick->LOAD - SysTick->VAL) / 6;
}

uint32_t SysTimeNs()
{
	return ((SysTick->LOAD - SysTick->VAL)) * 166;
}



