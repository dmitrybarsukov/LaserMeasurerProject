#include "stdint.h"
#include "si5351drv.h"
#include "i2cmemdriver.h"


void Si5351_SetLoadCap(uint8_t loadcap)
{
	I2CMemWriteByte(SI5351_ADDRESS, SI5351_XTAL_LOAD_CAP, loadcap);
}

void Si5351_EnableOutputs(uint8_t outputs)
{
	I2CMemWriteByte(SI5351_ADDRESS, SI5351_OUTCONTROL, ~outputs);
}

void Si5351_SetupPllSrc(uint8_t data)
{
	I2CMemWriteByte(SI5351_ADDRESS, SI5351_PLLxSRC, data);
}

void Si5351_ClkxConfig(uint8_t clkn, uint8_t data)
{
	uint8_t addr = SI5351_CLKxBASE + SI5351_CLKxOFFS * clkn;
	I2CMemWriteByte(SI5351_ADDRESS, addr, data);
}

void Si5351_ClkDisConfig(uint16_t data)
{
	I2CMemWriteByte(SI5351_ADDRESS, SI5351_CLK0123_DIS_STATE, data & 0xFF);
	I2CMemWriteByte(SI5351_ADDRESS, SI5351_CLK4567_DIS_STATE, data >> 8);
}

void Si5351_PllMSnRawConfig(uint8_t PllMSn, uint32_t param1, uint32_t param2, uint32_t param3)
{
	uint8_t addr = SI5351_PLL_MSx_BASE + SI5351_PLL_MSx_OFFS * PllMSn;
	uint8_t cfg[8];
	cfg[0] = param3 >> 8;
	cfg[1] = param3 & 0xFF;
	cfg[2] = (param1 >> 16) & 0x03;
	cfg[3] = param1 >> 8;
	cfg[4] = param1 & 0xFF;
	cfg[5] = ((param3 >> 12) & 0xF0) | ((param2 >> 16) & 0x0F);
	cfg[6] = param2 >> 8;
	cfg[7] = param2 & 0xFF;
	I2CMemWriteBlock(SI5351_ADDRESS, addr, cfg, 8);
}

void Si5351_MSnRawConfig(uint8_t MSn, uint32_t param1, uint32_t param2, uint32_t param3, uint8_t divider)
{
	uint8_t addr = SI5351_MSx_BASE + SI5351_MSx_OFFS * MSn;
	uint8_t cfg[8];
	cfg[0] = param3 >> 8;
	cfg[1] = param3 & 0xFF;
	cfg[2] = ((param1 >> 16) & 0x03) | divider;
	cfg[3] = param1 >> 8;
	cfg[4] = param1 & 0xFF;
	cfg[5] = ((param3 >> 12) & 0xF0) | ((param2 >> 16) & 0x0F);
	cfg[6] = param2 >> 8;
	cfg[7] = param2 & 0xFF;
	I2CMemWriteBlock(SI5351_ADDRESS, addr, cfg, 8);
}

void Si5351_PllMSnConfig(uint8_t PllMSn, uint32_t a, uint32_t b, uint32_t c)
{
	uint8_t addr = SI5351_PLL_MSx_BASE + SI5351_PLL_MSx_OFFS * PllMSn;
	uint8_t cfg[8];
	uint32_t param1, param2, param3;
	
	param3 = c;
	param2 = 128 * b - c * (uint32_t)(128.0 * b / c);
	param1 = 128 * a + (uint32_t)(128.0 * b / c) - 512;
	
	cfg[0] = param3 >> 8;
	cfg[1] = param3 & 0xFF;
	cfg[2] = (param1 >> 16) & 0x03;
	cfg[3] = param1 >> 8;
	cfg[4] = param1 & 0xFF;
	cfg[5] = ((param3 >> 12) & 0xF0) | ((param2 >> 16) & 0x0F);
	cfg[6] = param2 >> 8;
	cfg[7] = param2 & 0xFF;
	I2CMemWriteBlock(SI5351_ADDRESS, addr, cfg, 8);
}

void Si5351_MSnConfig(uint8_t MSn, uint32_t a, uint32_t b, uint32_t c, uint8_t divider)
{
	uint8_t addr = SI5351_MSx_BASE + SI5351_MSx_OFFS * MSn;
	uint8_t cfg[8];
	int32_t param1, param2, param3;
	
	param3 = c;
	param2 = 128 * b - c * (uint32_t)(128.0 * b / c);
	param1 = 128 * a + (uint32_t)(128.0 * b / c) - 512;
	
	cfg[0] = param3 >> 8;
	cfg[1] = param3 & 0xFF;
	cfg[2] = ((param1 >> 16) & 0x03) | divider;
	cfg[3] = param1 >> 8;
	cfg[4] = param1 & 0xFF;
	cfg[5] = ((param3 >> 12) & 0xF0) | ((param2 >> 16) & 0x0F);
	cfg[6] = param2 >> 8;
	cfg[7] = param2 & 0xFF;
	I2CMemWriteBlock(SI5351_ADDRESS, addr, cfg, 8);
}

void Si5351_ResetPLL(void)
{
	I2CMemWriteByte(SI5351_ADDRESS, SI5351_PLL_RESET, SI5351_PLLA_RST | SI5351_PLLB_RST);
}
