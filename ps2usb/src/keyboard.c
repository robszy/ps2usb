/*
 * keyboard.c
 *
 * Created: 2014-03-08 10:52:02
 *  Author: Robert Szymiec
 *	Copyright (c) 2015 Robert Szymiec <robert.szymiec@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */ 

#include "report.h"
#include "usbdrv.h"
#include "keyboard.h"
#include "oddebug.h"
#include "matrix.h"


typedef struct {
	uint8_t  report_id;
	uint16_t usage;
} __attribute__ ((packed)) report_extra_t;

static void send_system(uint16_t data)
{
	static uint16_t last_data = 0;
	if (data == last_data) return;
	last_data = data;

	report_extra_t report = {
		.report_id = REPORT_ID_SYSTEM,
		.usage = data
	};
	if (usbInterruptIsReady3()) {
		usbSetInterrupt3((void *)&report, sizeof(report));
	}
}

static void send_consumer(uint16_t data)
{
	static uint16_t last_data = 0;
	if (data == last_data) return;
	last_data = data;

	report_extra_t report = {
		.report_id = REPORT_ID_CONSUMER,
		.usage = data
	};
	if (usbInterruptIsReady3()) {
		usbSetInterrupt3((void *)&report, sizeof(report));
	}
}

report_keyboard_t keyboard_report;
uint8_t mods = 0;

void add_mods(uint8_t amods) { mods |= amods; }
void del_mods(uint8_t amods) { mods &= ~amods; }


static inline void add_key_byte(uint8_t code)
{
    int8_t i = 0;
    int8_t empty = -1;
    for (; i < REPORT_KEYS; i++) {
        if (keyboard_report.keys[i] == code) {
            break;
        }
        if (empty == -1 && keyboard_report.keys[i] == 0) {
            empty = i;
        }
    }
    if (i == REPORT_KEYS) {
        if (empty != -1) {
            keyboard_report.keys[empty] = code;
        }
    }
}

static inline void del_key_byte(uint8_t code)
{
    for (uint8_t i = 0; i < REPORT_KEYS; i++) {
        if (keyboard_report.keys[i] == code) {
            keyboard_report.keys[i] = 0;
        }
    }
}

void clear_keys(void)
{
    // not clear mods
    for (int8_t i = 1; i < REPORT_SIZE; i++) {
        keyboard_report.raw[i] = 0;
    }
}


/*
 * Utilities for actions.
 */
void sendRemoteWakeUp(void){
	cli();
	uint8_t ddrd = DDRD, ddr_init = DDRD; // read direction
	DBG2(0x0f,0,0);
	ddrd |= USBMASK; // set output for D+ and D-
	USBOUT |= (1<< USBMINUS); // set J state in port
	
	USBDDR = ddrd; // acquire bus
	// set k state
	USBOUT ^= USBMASK;
	// wait
	_delay_ms(10);
	// set idle
	USBOUT ^= USBMASK;
	// revert ddr
	USBDDR = ddr_init;
	// set port without pullup ie D+,D- = 0
	USBOUT &= ~( 1 << USBMINUS );
	sei();
	
}
extern uint8_t suspendMode;

#define PS2_USART_INIT() do {   \
UCSR0C = ((1 << UMSEL00) |  \
(3 << UPM00)   |  \
(0 << USBS0)   |  \
(3 << UCSZ00)  |  \
(0 << UCPOL0));   \
UCSR0A = 0;                 \
UBRR0H = 0;                 \
UBRR0L = 0;                 \
} while (0)
#define PS2_USART_RX_INT_ON() do {  \
    UCSR0B = ((1 << RXCIE0) |       \
              (1 << RXEN0));        \
} while (0)

void trunOffMouseRx(void){
	// turn off mouse rx
	UCSR0B &= ~(1 << RXEN0);
	// set 8 bit no parity and ... (defaults)
	UCSR0C = ( 3 << UCSZ00);
	odDebugInit(1);
}
void trunOnMouseRx(void){
	// dumy usart procedure
	odDebugInit(2);
	PS2_USART_INIT();
	PS2_USART_RX_INT_ON();
}

void register_code(uint8_t code)
{
    if (code == KC_NO) {
        return;
    }

    else if IS_KEY(code) {
		if (code == KC_P && suspendMode == 1 ){
			trunOffMouseRx();
			printDump();
			waitForEnd();
			trunOnMouseRx();
		}			
	    add_key_byte(code);
	    send_keyboard_report();
    }
    else if IS_MOD(code) {
        add_mods(MOD_BIT(code));
        send_keyboard_report();
    }
    else if IS_SYSTEM(code) {
		if (code == KC_WAKE){
			if ( suspendMode == 1 ){
				//trunOnMouseRx();
				sendRemoteWakeUp();
				suspendMode = 0; // when rWkup sent we are sure that suspend is cleared
			} else
				DBG2(0x06,0,0);
		}
		else if (code == KC_PWR && suspendMode == 0 )
			send_system(KEYCODE2SYSTEM(code));
		else if (code == KC_SLEP && suspendMode == 0)
			send_system(KEYCODE2SYSTEM(code));
	}       
    //else if IS_CONSUMER(code) {
        //host_consumer_send(KEYCODE2CONSUMER(code)); TODO: consumer keys to change volume etc.
    //}
}

void unregister_code(uint8_t code)
{
    if (code == KC_NO) {
        return;
    }
    else if IS_KEY(code) {
        del_key_byte(code);
        send_keyboard_report();
    }
    else if IS_MOD(code) {
        del_mods(MOD_BIT(code));
        send_keyboard_report();
    }
    else if IS_SYSTEM(code) {
	    send_system(0);
    }
    //else if IS_CONSUMER(code) {
	    //host_consumer_send(0);
	//}
}





/* Keyboard report send buffer */
#define KBUF_SIZE 16
static report_keyboard_t kbuf[KBUF_SIZE];
static uint8_t kbuf_head = 0;
static uint8_t kbuf_tail = 0;
extern uint8_t inhibit;
extern schar usbRxLen;


/* transfer keyboard report from buffer */
void vusb_transfer_keyboard(void)
{
    if (usbInterruptIsReady() && ! inhibit) {
        if (kbuf_head != kbuf_tail) {
            usbSetInterrupt((void *)&kbuf[kbuf_tail], sizeof(report_keyboard_t));
            kbuf_tail = (kbuf_tail + 1) % KBUF_SIZE;
            //if (debug_keyboard) {
                //print("V-USB: kbuf["); pdec(kbuf_tail); print("->"); pdec(kbuf_head); print("](");
                //phex((kbuf_head < kbuf_tail) ? (KBUF_SIZE - kbuf_tail + kbuf_head) : (kbuf_head - kbuf_tail));
                //print(")\n");
            //}
        }
    }// else if (inhibit)
		//usbTxLen1 = USBPID_NAK;
}

void send_keyboard(report_keyboard_t *report)
{
    uint8_t next = (kbuf_head + 1) % KBUF_SIZE;
    if (next != kbuf_tail) {
        kbuf[kbuf_head] = *report;
        kbuf_head = next;
    } else {
        DBG2(0x22,0,0);
    }
}

void send_keyboard_report(){
	keyboard_report.mods = mods;
	send_keyboard(&keyboard_report);
}

