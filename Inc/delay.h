#ifndef _DELAY_H
#define _DELAY_H

#include "stdint.h"

void DelayMs(uint32_t t);
void DelayUs(uint32_t t);

uint32_t SysTimeS(void);
uint32_t SysTimeMs(void);
uint32_t SysTimeUs(void);

#endif // _DELAY_H
