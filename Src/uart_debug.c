#include "stm32f0xx_ll_usart.h"
#include "stdio.h"

int fputc(int ch, FILE *f) {
	USART1->TDR = ch;
	while(!(USART1->ISR & LL_USART_ISR_TXE));
	return 0;
}

int fgetc(FILE * filestream)
{
	if(USART1->ISR & LL_USART_ISR_RXNE)
		return USART1->RDR;
	else
		return -1;
}
