#ifndef _SOFT_I2C
#define _SOFT_I2C

#include "stdint.h"

#define I2C_READ 1
#define I2C_WRITE 0

void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WriteAddress(uint8_t address, uint8_t rw);
uint8_t I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(void);

void I2C_Delay(void);

#endif // _SOFT_I2C
