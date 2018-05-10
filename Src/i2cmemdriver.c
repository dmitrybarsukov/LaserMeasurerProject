#include "i2cmemdriver.h"
#include "softi2c.h"
#include "stdint.h"

uint8_t I2CMemWriteByte(uint8_t devAddress, uint8_t address, uint8_t data)
{
	int error = 0;
	I2C_Start();
	
	if(!I2C_WriteAddress(devAddress, I2C_WRITE))
		error = 1;
	
	if(!I2C_WriteByte(address))
		error = 1;
	
	if(!I2C_WriteByte(data))
		error = 1;
	
	I2C_Stop();
	return !error;
}

uint8_t I2CMemWriteBlock(uint8_t devAddress, uint8_t address, uint8_t* data, uint8_t length)
{
	int error = 0;
	I2C_Start();
	
	if(!I2C_WriteAddress(devAddress, I2C_WRITE))
		error = 1;
	
	if(!I2C_WriteByte(address))
		error = 1;
		
	while(length-- > 0)
		if(!I2C_WriteByte(*data++))
			error = 1;
		
	I2C_Stop();
	return !error;
}

uint8_t I2CMemReadByte(uint8_t devAddress, uint8_t address)
{
	return 0;
}

uint8_t I2CMemReadBlock(uint8_t devAddress, uint8_t* address, uint8_t length)
{
	return 0;
}
