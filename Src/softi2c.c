#include "softi2c.h"
#include "stdint.h"
#include "platform.h"
#include "delay.h"

void I2C_Start(void)
{
	SdaPinWrite(1);
	I2C_Delay();
	SclPinWrite(1);
	I2C_Delay();
	while(!SdaPinRead())
	{
		SclPinWrite(0);
		I2C_Delay();
		SclPinWrite(1);
		I2C_Delay();
	}
	SdaPinWrite(0);
	I2C_Delay();
	SclPinWrite(0);
}

void I2C_Stop(void)
{
	SdaPinWrite(0);
	I2C_Delay();
	SclPinWrite(1);
	I2C_Delay();
	SdaPinWrite(1);
	I2C_Delay();
}

uint8_t I2C_WriteAddress(uint8_t address, uint8_t rw)
{
	return I2C_WriteByte(address << 1 | (rw ? 0x01 : 0x00));
}

uint8_t I2C_WriteByte(uint8_t data)
{
	uint8_t ack;
	for(int i = 0; i < 8; i++)
	{
		SdaPinWrite((data << i) & 0x80);
		I2C_Delay();
		SclPinWrite(1);
		I2C_Delay();       
		SclPinWrite(0);
		//data <<= 1; 
	}
	SdaPinWrite(1);
	I2C_Delay();
	ack = !SdaPinRead();
	SclPinWrite(1);
	I2C_Delay();       
	SclPinWrite(0);
	SdaPinWrite(0);
	return ack;
}

uint8_t I2C_ReadByte(void)
{
	return 0;
}


void I2C_Delay(void)
{
	int i = 8;
	while(i--);
	return;
}
