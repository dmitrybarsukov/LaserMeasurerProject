#ifndef _SI5351_DRV
#define _SI5351_DRV

#include "stdint.h"

#define SI5351_ADDRESS								0x60

#define SI5351_STATUS									0
#define SI5351_INTSTATUS							1
#define SI5351_INTMASK								2
#define SI5351_OUTCONTROL							3
#define SI5351_OUTMASK								9
#define SI5351_PLLxSRC								15

#define SI5351_PLLxSRC_DIV1						0x00
#define SI5351_PLLxSRC_DIV2						0x40
#define SI5351_PLLxSRC_DIV4						0x80
#define SI5351_PLLxSRC_DIV8						0xC0

#define SI5351_PLLASRC_XTAL						0x00
#define SI5351_PLLASRC_CLKIN					0x04
#define SI5351_PLLBSRC_XTAL						0x00
#define SI5351_PLLBSRC_CLKIN					0x08

#define SI5351_CLKxBASE								16
#define SI5351_CLKxOFFS								1

#define SI5351_CLKx_OFF								0x80
#define SI5351_CLKx_ON								0x00
#define SI5351_CLKx_MSx_Integer				0x40
#define SI5351_CLKx_MSx_Fractional		0x00
#define SI5351_CLKx_MSx_SRC_PLLB			0x20
#define SI5351_CLKx_MSx_SRC_PLLA			0x00
#define SI5351_CLKx_INVERT						0x10
#define SI5351_CLKx_NOINVERT					0x00
#define SI5351_CLKx_SRC_XTAL					0x00
#define SI5351_CLKx_SRC_CLKIN					0x04
#define SI5351_CLKx_SRC_MSx						0x0C
#define SI5351_CLKx_IDRV_2ma					0x00
#define SI5351_CLKx_IDRV_4ma					0x01
#define SI5351_CLKx_IDRV_6ma					0x02
#define SI5351_CLKx_IDRV_8ma					0x03

#define SI5351_CLK0123_DIS_STATE			24
#define SI5351_CLK4567_DIS_STATE			25

#define SI5351_CLKx_DIS_STATE_LOW			0x00
#define SI5351_CLKx_DIS_STATE_HIGH		0x01
#define SI5351_CLKx_DIS_STATE_HIZ			0x02
#define SI5351_CLKx_DIS_STATE_NODIS		0x03

#define SI5351_PLL_MSx_BASE						26
#define SI5351_PLL_MSx_OFFS						8

#define SI5351_MSx_BASE								42
#define SI5351_MSx_OFFS								8

#define SI5351_MSx_DIV1								0x00
#define SI5351_MSx_DIV2								0x10
#define SI5351_MSx_DIV4								0x20
#define SI5351_MSx_DIV8								0x30
#define SI5351_MSx_DIV16							0x40
#define SI5351_MSx_DIV32							0x50
#define SI5351_MSx_DIV64							0x60
#define SI5351_MSx_DIV128							0x70
#define SI5351_MSx_DIVBY4							0x0C
#define SI5351_MSx_DIVBYOTHER					0x00

#define SI5351_PLL_RESET							177

#define SI5351_PLLA_RST								0x20
#define SI5351_PLLB_RST								0x80

#define SI5351_XTAL_LOAD_CAP					183

#define SI5351_XTAL_LOAD_CAP_6pf			0x40
#define SI5351_XTAL_LOAD_CAP_8pf			0x80
#define SI5351_XTAL_LOAD_CAP_10pf			0xC0

void Si5351_SetLoadCap(uint8_t loadcap);
void Si5351_EnableOutputs(uint8_t outputs);
void Si5351_SetupPllSrc(uint8_t data);
void Si5351_ClkxConfig(uint8_t clkn, uint8_t data);
void Si5351_ClkDisConfig(uint16_t data);
void Si5351_PllMSnRawConfig(uint8_t PllMSn, uint32_t param1, uint32_t param2, uint32_t param3);
void Si5351_MSnRawConfig(uint8_t MSn, uint32_t param1, uint32_t param2, uint32_t param3, uint8_t divider);
void Si5351_PllMSnConfig(uint8_t PllMSn, uint32_t a, uint32_t b, uint32_t c);
void Si5351_MSnConfig(uint8_t MSn, uint32_t a, uint32_t b, uint32_t c, uint8_t divider);
void Si5351_ResetPLL(void);











#endif // _SI5351_DRV
