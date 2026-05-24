#include <avr/io.h>
#include <avr/interrupt.h>
#include "UserInterface.h"
#include "FreeRTOSBufferedSerial.h"
#include "FreeRTOS.h"
#include "SerialInterface.h"

#ifndef OS_AVRX

void put_char(USART_t * usart, char c)	// Blocking output
{
	while(!(usart->STATUS & USART_DREIF_bm));
	usart->DATA=c;
	
	while(!(usart->STATUS & USART_TXCIF_bm));
	usart->STATUS |= USART_DREIF_bm;
}

#endif /* OS_FREERTOS */