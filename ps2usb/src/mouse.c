/*
 * mouse.c
 *
 * Created: 2014-06-04 11:49:51
 *  Author: Robert Szymiec
 
 
 Copyright (c) 2015 Robert Szymiec <robert.szymiec@gmail.com>

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
#include "ps2_usart.h"
#include "oddebug.h"
#include "report.h"
#include "usbdrv.h"


#include "mouse.h"

uint8_t mod3BytesCounter;

/* supports only 3 button mouse at this time + roll */
uint8_t ps2_mouse_init(void) {
	uint8_t rcv;

	ps2_host_init_mouse();
sei();
	   // wait for powering up
_delay_ms(1000);
	// send Reset
	rcv = ps2_host_send_mouse(0xFF);
	DBG2(0x40,&rcv,1);
	// read bat completion
	rcv = ps2_host_recv_response_mouse();
	DBG2(0x41,&rcv,1);
	// read Device ID
	rcv = 99;
	rcv = ps2_host_recv_response_mouse();
	DBG2(0x42,&rcv,1);
	// sampling rate
	rcv = ps2_host_send_mouse(0xF3);
	DBG2(0x43,&rcv,1);
	rcv = ps2_host_send_mouse(0xC8);
	DBG2(0x44,&rcv,1);
	rcv = ps2_host_send_mouse(0xF3);
	DBG2(0x45,&rcv,1);
	rcv = ps2_host_send_mouse(0x64);
	DBG2(0x46,&rcv,1);
	rcv = ps2_host_send_mouse(0xF3);
	DBG2(0x47,&rcv,1);
	rcv = ps2_host_send_mouse(0x50);
	DBG2(0x48,&rcv,1);
	// read device type
	rcv = ps2_host_send_mouse(0xF2);
	DBG2(0x49,&rcv,1);
	rcv = ps2_host_recv_response_mouse();
	DBG2(0x4a,&rcv,1);
	
	rcv = ps2_host_send_mouse(0xF3);
	DBG2(0x4b,&rcv,1);
	rcv = ps2_host_send_mouse(0x64);
	
	rcv = ps2_host_send_mouse(0xE8);
	DBG2(0x4c,&rcv,1);
	rcv = ps2_host_send_mouse(0x03);
	DBG2(0x4d,&rcv,1);
	// send Enable device 
	rcv = ps2_host_send_mouse(0xF4);
	DBG2(0x4e,&rcv,1);
	mod3BytesCounter = 0;
	//while (1){
	//mouse_task();
	//_delay_us(10);
	//}	
	// stop logging
	
	odDebugInit(2);
cli();
	return 0;
}


void mouse_task(void){
	
	report_mouse_t mouse_report;
	if (mod3BytesCounter >= 4){
		DBG2(0x44,0,0);
		mouse_report.buttons = ps2_host_recv_response_mouse();
		DBG2(0x43,&mouse_report.buttons,1);
		mouse_report.x = ps2_host_recv_response_mouse();
		DBG2(0x43,&mouse_report.x,1);
		mouse_report.y = ps2_host_recv_response_mouse();
		DBG2(0x43,&mouse_report.y,1);
		mouse_report.v = -((char)ps2_host_recv_response_mouse());
		DBG2(0x43,&mouse_report.v,1);
		
		mouse_report.x = X_IS_NEG ?
		((!X_IS_OVF && -127 <= mouse_report.x && mouse_report.x <= -1) ?  mouse_report.x : -127) :
		((!X_IS_OVF && 0 <= mouse_report.x && mouse_report.x <= 127) ? mouse_report.x : 127);
		mouse_report.y = Y_IS_NEG ?
		((!Y_IS_OVF && -127 <= mouse_report.y && mouse_report.y <= -1) ?  mouse_report.y : -127) :
		((!Y_IS_OVF && 0 <= mouse_report.y && mouse_report.y <= 127) ? mouse_report.y : 127);
		mouse_report.buttons &= PS2_MOUSE_BTN_MASK;
		mouse_report.h =0;
		mouse_report.y = -mouse_report.y;
		
		mod3BytesCounter -= 4;
		send_mouse(&mouse_report);
		//DBG3(0x1,0,0);
	}
}
/* Mouse report send buffer */
#define MBUF_SIZE 16
static report_mouse_t mbuf[MBUF_SIZE];
static uint8_t mbuf_head = 0;
static uint8_t mbuf_tail = 0;

void send_mouse(report_mouse_t *report)
{
	uint8_t next = (mbuf_head + 1) % MBUF_SIZE;
	if (next != mbuf_tail) {
		mbuf[mbuf_head] = *report;
		mbuf_head = next;
	} else {
		DBG2(0x23,0,0);
	}
}

void vusb_transfer_mouse(void)
{
	if (usbInterruptIsReady3()) {
		if (mbuf_head != mbuf_tail) {
			vusb_mouse_report_t r = {
				.report_id = REPORT_ID_MOUSE,
				.report = mbuf[mbuf_tail]
			};
			usbSetInterrupt3((void *)&r, sizeof(vusb_mouse_report_t));
			mbuf_tail = (mbuf_tail + 1) % MBUF_SIZE;
		}
	}// else if (inhibit)
	//usbTxLen1 = USBPID_NAK;
}
