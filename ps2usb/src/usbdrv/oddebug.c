/* Name: oddebug.c
 * Project: AVR library
 * Author: Christian Starkjohann
 * Creation Date: 2005-01-16
 * Tabsize: 4
 * Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

#include "oddebug.h"
#include "usbdrv.h"


void (*uartPutcPtr)()=0;


#if DEBUG_LEVEL > 0

#warning "Never compile production devices with debugging enabled"

static void uartPutc(char c)
{
    while(!(ODDBG_USR & (1 << ODDBG_UDRE)));    /* wait for data register empty */
    ODDBG_UDR = c;
	UCSR0A |= (1 << TXC0);
}

void waitForEnd(void){
	while(!(ODDBG_USR & (1 << TXC0)));
}	
void uartPutcProg( uint8_t c )
{
	//cli();
	c = ~c;
	STX_PORT &= ~(1<<STX_BIT);            // start bit
	for( uint8_t i = 10; i; i-- ){        // 10 bits
		_delay_us( 1e6 / BAUD );            // bit duration
		if( c & 1 )
			STX_PORT &= ~(1<<STX_BIT);        // data bit 0
		else
			STX_PORT |= 1<<STX_BIT;           // data bit 1 or stop bit
		c >>= 1;
	}
	
	
	//waitForIdle();
	//_delay_us(5000);
	//USB_INTR_PENDING = ( 1 << USB_INTR_PENDING_BIT );
	//sei();
	//
} 

void uartPutcDummy( uint8_t c ){}

static uchar    hexAscii(uchar h)
{
    h &= 0xf;
    if(h >= 10)
        h += 'a' - (uchar)10 - '0';
    h += '0';
    return h;
}

static void printHex(uchar c)
{
    (*uartPutcPtr)(hexAscii(c >> 4));
    (*uartPutcPtr)(hexAscii(c));
}

void    odDebug(uchar prefix, uchar *data, uchar len)
{
    printHex(prefix);
    (*uartPutcPtr)(':');
    while(len--){
        (*uartPutcPtr)(' ');
        printHex(*data++);
    }
    (*uartPutcPtr)('\r');
    (*uartPutcPtr)('\n');
	
}
void  odDebugInit(uchar usart)
{
	// no usart version
	//DDRD |= ( 1 << STX_BIT );
	//STX_PORT |= ( 0 << STX_BIT );
	if (usart == 1) {
		DDRD &= ~( 1 << STX_BIT );
		STX_PORT &= ~( 1 << STX_BIT );//ddr i port na zero nie potrzebne o ile nie u¿ywamy progputc
		ODDBG_UCR |= (1<<ODDBG_TXEN); // tx enable
		ODDBG_UBRR = 10; //F_CPU / (19200 * 16L) - 1;
		uartPutcPtr = uartPutc;
	} else if (usart == 0) {
		ODDBG_UCR &= ~(1<<ODDBG_TXEN); // tx disable
		DDRD |= ( 1 << STX_BIT ); // output na tx
		STX_PORT |= ( 1 << STX_BIT ); // 1 na tx
		uartPutcPtr = uartPutcProg;
	} else {
		uartPutcPtr = uartPutcDummy;
	}
}

#endif
