#ifndef _I2C_MEMDRIVER
#define _I2C_MEMDRIVER

#include "softi2c.h"
#include "stdint.h"

uint8_t I2CMemWriteByte(uint8_t devAddress, uint8_t address, uint8_t data);
uint8_t I2CMemWriteBlock(uint8_t devAddress, uint8_t address, uint8_t* data, uint8_t length);
uint8_t I2CMemReadByte(uint8_t devAddress, uint8_t address);
uint8_t I2CMemReadBlock(uint8_t devAddress, uint8_t* address, uint8_t length);

#endif // _I2C_MEMDRIVER
