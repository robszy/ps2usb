/*
Copyright 2015 Robert Szymiec <robert.szymiec@gmail.com>
Copyright 2010,2011,2012,2013 Jun WAKO <wakojun@gmail.com>

This software is licensed with a Modified BSD License.
All of this is supposed to be Free Software, Open Source, DFSG-free,
GPL-compatible, and OK to use in both free and proprietary applications.
Additions and corrections to this file are welcome.


Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.

* Neither the name of the copyright holders nor the names of
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * PS/2 protocol Pin interrupt version
 */

#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "ps2.h"
#include "oddebug.h"
#include "usbdrv.h"




#define WAIT(stat, us, err) do { \
    if (!wait_##stat(us)) { \
        ps2_error = err; \
        goto ERROR; \
    } \
} while (0)


volatile uint8_t ps2_error = PS2_ERR_NONE;
volatile uint8_t ps2_error_data = PS2_ERR_NONE;
volatile uint8_t	ps2_err_state = 0 ;
uint8_t sendMode = 0;
uint8_t dataToSend = 0;
uint8_t data = 0;
uint8_t parity = 1;
uint8_t bitNumToSend = 0;
uint8_t timingFirstBit = 0;
uint8_t sendError = 0;

uint8_t inhibit = 0;
volatile uint8_t requestBit = 0;
volatile uint8_t requestBitPrev = 0;

uint16_t timeSamlpes[12] = {0};
uint8_t timeSamlpes8[12] = {0};
enum state_enum {
	INIT,
	START,
	BIT0, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7,
	PARITY,
	STOP,
	INV = 0x80
} state = INIT;


static inline uint8_t pbuf_dequeue(void);
static inline void pbuf_enqueue(uint8_t data);
static inline bool pbuf_has_data(void);
static inline void pbuf_clear(void);


void ps2_host_init(void)
{
    //idle();
    PS2_INT_INIT();
    PS2_INT_ON();
	uint8_t resp;
	sei();
	// send reset
	ps2_host_send(0xFF);
	//if ( timingFirstBit ){
		//DBG2(0x75,&timingFirstBit,1);
		//timingFirstBit = 0;
	//}
	if ( ps2_error ){
		DBG2(0x71,&ps2_error,1);
		ps2_error = 0;
		ps2_err_state = 0;
	}
	resp = ps2_host_recv_response();
	if (resp != PS2_ACK)
		DBG2(0x72,&resp,1);
	
    // POR(150-2000ms) plus BAT(300-500ms) may take 2.5sec([3]p.20)
    _delay_ms(2500);
	cli();
}

void init_ps2_host_send(uint8_t data){
	ps2_error = PS2_ERR_NONE;

	PS2_INT_OFF();

	/* terminate a transmission if we have */
	clock_lo();
	_delay_us(100); // 100us [4]p.13, [5]p.50

	/* 'Request to Send' and Start bit */
	data_lo();
	clock_release();
	sendMode = 1;
	// set first bit to send
	bitNumToSend = 0;
	dataToSend = data;
	
	EIFR = (1<<1);
	PS2_INT_ON();
}

uint8_t ps2_host_send(uint8_t data)
{
	uint8_t i;
	for (i =0 ;i < 12;i++)
	{
		timeSamlpes8[i]=0;
	}
	init_ps2_host_send(data);
	
	while (sendMode) {
		usbPoll();
		_delay_ms(1); 
	}		
		
	_delay_ms(3);
	
	return 0;
}

uint8_t ps2_host_recv_response(void)
{
    // Command may take 25ms/20ms at most([5]p.46, [3]p.21)
    uint8_t retry = 25;
    while (retry-- && !pbuf_has_data()) {
		usbPoll();
        _delay_ms(1);
    }
    return pbuf_dequeue();
}


/* get data received by interrupt */
uint8_t ps2_host_recv(void)
{	
    if (pbuf_has_data()) {
        //ps2_error = PS2_ERR_NONE;
        return pbuf_dequeue();
    } else {
        //ps2_error = PS2_ERR_NODATA;
        return 0;
    }
}
// handling one bit at a time
void handleBit(bool dataIn){
	
	if (ps2_err_state)
		return;// return to not overwrite debug data
	state++;
	switch (state) {
		case START:
			//DBG2(0x54,0,0);
			inhibit = 1;
			//TCNT1 = 0;
			//USB_INT_OFF();
			//TCCR1B = ( 1 << WGM12 | 1<< CS10 );
			if ( dataIn ){
				goto ERROR;
			}
			break;
		case BIT0:
		case BIT1:
		case BIT2:
		case BIT3:
		case BIT4:
		case BIT5:
		case BIT6:
		case BIT7:
			//timeSamlpes[state] = TCNT1;
			data >>= 1;
			if ( dataIn ) {
				data |= 0x80;
				parity++;
			}
			break;
		case PARITY:
			//timeSamlpes[state] = TCNT1;
			if ( dataIn ) {
				if (!(parity & 0x01))
					goto ERROR;
			} else {
				if (parity & 0x01)
					goto ERROR;
			}
			break;
		case STOP:
			//timeSamlpes[state] = TCNT1;
			//TCCR1B = ( 1 << WGM12 | 0<< CS10 );
			if ( !dataIn )
				goto ERROR;
		
			//DBG2(0x66,&numEnt,1);
		
			pbuf_enqueue(data);
			goto DONE;
			break;
		default:
			goto ERROR;
	}
	goto RETURN;
ERROR:
	//TCCR1B = ( 1 << WGM12 | 0<< CS10 );
	ps2_error = state;
	ps2_error_data = data;
	ps2_err_state = 1;
DONE:
	//TCCR1B = ( 1 << WGM12 | 0<< CS10 );
	inhibit = 0;
	//USB_INT_ON();
	state = INIT;
	data = 0;
	parity = 1;
	requestBit = 0;
RETURN:
	return;
}

void sendBit(bool fromPcInt ){
	// first check then action
	
	if ((bitNumToSend == 0) && (fromPcInt /*|| (debug == 1)*/)){ // if 0 bit turns up from pcint we don't know if timing is right
		//resetSend();					  // so from begining
		//DBG2(0x95,0,0);		
		TCNT0 = 0;
		TCCR0B = ((  1<< CS00 )| (1 << CS01));
		sendError = 1;  
	} else if (bitNumToSend == 0){ 
		// if not from pcint
		// start timer because we don't know how much time goes from release clk to first falling edge
		// turn on timing to first bit
		TCNT0 = 0;
		TCCR0B = ((  1<< CS00 )| (1 << CS01));
		//debug++;
	} else if (TCNT0 >  28  ){
		// we dont have transmission of bit for minimum 26 = 83 us + delay 2* 3,2 = 6,4 us
		// timer resolution 3.2 us
		
		
		//DBG2(0x75,&bitNumToSend,1);
		//bitNumToSend = 0;
		//resetSend();
		sendError = 1;
	}
	if (!sendError)
		timeSamlpes8[bitNumToSend] = TCNT0;
	TCNT0 = 0;
	
	if ((sendError == 1) && bitNumToSend != 10 ){
		if (bitNumToSend == 8) {
			// at partity bit try to invalidate data by setting inverted partiy bit
			if (parity) { data_lo(); } else { data_hi(); }
		} else
			data_lo();
		bitNumToSend++;
		return;
	}
	
	if (bitNumToSend <= 7) {
		if ( dataToSend &(1<<bitNumToSend)) {
			parity = !parity;
			data_hi();
		} else {
			data_lo();
		}
		
		
	} else if (bitNumToSend == 8){
		// send parity
		if (parity) { data_hi(); } else { data_lo(); }
		
	} else if (bitNumToSend == 9){
		// send stop bit
		data_hi();
		
	} else if ( bitNumToSend == 10 ){
		// check for ack
		data_release();
		if (data_in())
			ps2_error = 10;
		// clear vars as transmission ended
		//DBG2(0x96,&bitNumToSend,1);
		sendMode = 0;
		dataToSend = 0;
		sendError = 0; // close potentially skewed transmission
		data = 0;
		parity = 1;
		// stop T0
		TCCR0B =  0 ;
	
	}
	bitNumToSend++;
}

ISR(PS2_INT_VECT)
{
	bool dataIn;
	if (!sendMode) {
		// return unless falling edge
		dataIn = data_in();
	
		sei();
		handleBit(dataIn);
	
	} else  //send data at falling edge of clk
		sendBit(false);
    return;
}


ISR(PCINT1_vect)
{
	PORTC &= ~1; //clear one written by asm at zero bit
	PCIFR = 2; // clear interrupt flag
	USB_INTR_ENABLE |= (1 << USB_INTR_ENABLE_BIT); //enable usb interrupts after handling ps2
	// do the work
	
	// if we are at receiving state 
	if (!sendMode){
		if (   requestBitPrev & 16   ){ // if r22 overwritten so 2 samples available
			handleBit(requestBit & (1 << PS2_DATA_BIT));
			handleBit( requestBitPrev & ( 1 << (PS2_DATA_BIT+1) ) ); // +1 because we shift when we hange state
			//handleBit(0);
			//handleBit(0); // debug code to get some errrors
			// state is in r22
			//requestBit = 0;
			//DBG2(0x75,0,0);
			
		} else if ( requestBitPrev  & 4 ){ // if we have any (one) sample
			//if (state == INIT && ((requestBit & (1 << PS2_DATA_BIT)) != 0 ))
				//DBG2(0xea,&requestBitPrev,1);
			handleBit(requestBit & (1 << PS2_DATA_BIT));
			//handleBit(0);
			//DBG2(0x78,&requestBitPrev,1);
			//ps2_err_state = state 
			//requestBit = 0;
			
		} else if (requestBitPrev != 0 ){
			DBG2(0xee,&requestBitPrev,1);
		}
		requestBitPrev = 0;
		if ( EIFR & 2 ){
			handleBit(data_in());
			EIFR  = 2;
		}		
	} else {
		sendBit(true);
	}
	return;
}

/* send LED state to keyboard */
void ps2_host_set_led(uint8_t led)
{
	uint8_t retryCount = 5;
	uint8_t resp = 0;
	
	while (retryCount--){
		sendError = 0;
		ps2_host_send(0xED);
		//if ( timingFirstBit ){
			//DBG2(0x75,&timingFirstBit,1);
			//timingFirstBit = 0;
		//}
		if ( ps2_error ){
			DBG2(0x71,&ps2_error,1);
			ps2_error = 0;
			ps2_err_state = 0;
		}
		resp = ps2_host_recv_response();
		if (resp != PS2_ACK)
			DBG2(0x72,&resp,1);
		if ( resp == PS2_ACK )
			break;		
	}	
	//DBG2(0x78,0,0);
	retryCount = 5;
	while (retryCount--){
		sendError = 0;
		ps2_host_send(led);
		if ( ps2_error ){
			DBG2(0x73,&ps2_error,1);
			ps2_error = 0;
			ps2_err_state = 0; 
		}
	
		resp = ps2_host_recv_response();
		if ( resp != PS2_ACK )
			DBG2(0x74,&resp,1);
		if ( resp == PS2_ACK )
			break;
	}
}


/*--------------------------------------------------------------------
 * Ring buffer to store scan codes from keyboard
 *------------------------------------------------------------------*/
#define PBUF_SIZE 32
static uint8_t pbuf[PBUF_SIZE];
static uint8_t pbuf_head = 0;
static uint8_t pbuf_tail = 0;
static inline void pbuf_enqueue(uint8_t data)
{
    uint8_t sreg = SREG;
    cli();
    uint8_t next = (pbuf_head + 1) % PBUF_SIZE;
    if (next != pbuf_tail) {
        pbuf[pbuf_head] = data;
        pbuf_head = next;
    } else {
        //print("pbuf: full\n");
    }
    SREG = sreg;
}
static inline uint8_t pbuf_dequeue(void)
{
    uint8_t val = 0;

    uint8_t sreg = SREG;
    cli();
    if (pbuf_head != pbuf_tail) {
        val = pbuf[pbuf_tail];
        pbuf_tail = (pbuf_tail + 1) % PBUF_SIZE;
    }
    SREG = sreg;

    return val;
}
static inline bool pbuf_has_data(void)
{
    uint8_t sreg = SREG;
    cli();
    bool has_data = (pbuf_head != pbuf_tail);
    SREG = sreg;
    return has_data;
}
static inline void pbuf_clear(void)
{
    uint8_t sreg = SREG;
    cli();
    pbuf_head = pbuf_tail = 0;
    SREG = sreg;
}

