/* Name: main.c
 * Project: ps2usb translator
 * Author: Robert Szymiec
 * Creation Date: 2014-01-05
 * Tabsize: 4
 * Copyright: (c) 2015 by Robert Szymiec
 * License: GNU GPL v2 or any later
 */

/*
This code was developed on atmega328 an should but it could be ported to any avr.
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */

#include "timer.h"
#include "keyboard.h"
#include "keycode.h"
#include "report.h"
#include "ps2.h"
#include "matrix.h"
#include "mouse.h"
/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */







//PROGMEM const char usbHidReportDescriptor[52] = { /* USB report descriptor, size must match usbconfig.h */
    //0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    //0x09, 0x02,                    // USAGE (Mouse)
    //0xa1, 0x01,                    // COLLECTION (Application)
    //0x09, 0x01,                    //   USAGE (Pointer)
    //0xA1, 0x00,                    //   COLLECTION (Physical)
    //0x05, 0x09,                    //     USAGE_PAGE (Button)
    //0x19, 0x01,                    //     USAGE_MINIMUM
    //0x29, 0x03,                    //     USAGE_MAXIMUM
    //0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    //0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    //0x95, 0x03,                    //     REPORT_COUNT (3)
    //0x75, 0x01,                    //     REPORT_SIZE (1)
    //0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    //0x95, 0x01,                    //     REPORT_COUNT (1)
    //0x75, 0x05,                    //     REPORT_SIZE (5)
    //0x81, 0x03,                    //     INPUT (Const,Var,Abs)
    //0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    //0x09, 0x30,                    //     USAGE (X)
    //0x09, 0x31,                    //     USAGE (Y)
    //0x09, 0x38,                    //     USAGE (Wheel)
    //0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    //0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
    //0x75, 0x08,                    //     REPORT_SIZE (8)
    //0x95, 0x03,                    //     REPORT_COUNT (3)
    //0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    //0xC0,                          //   END_COLLECTION
    //0xC0,                          // END COLLECTION
//};
/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */
typedef struct{
    uchar   buttonMask;
    char    dx;
    char    dy;
    char    dWheel;
	char	horizWheel;
}report_t;

static report_t reportBuffer;
static int      sinus = 7 << 6, cosinus = 0;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */
uchar vusb_keyboard_leds = 0;
uchar keyboard_leds = 0;


/* ------------------------------------------------------------------------- */

/*------------------------------------------------------------------*
 * Request from host                                                *
 *------------------------------------------------------------------*/
static struct {
    uint16_t        len;
    enum {
        NONE,
        SET_LED
    }               kind;
} last_req;

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    /* The following requests are never used. But since they are required by
     * the specification, we implement them in this example.
     */
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        //DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            usbMsgPtr = (void *)&reportBuffer;
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
			if (rq->wValue.word == 0x0200 && rq->wIndex.word == 0) {
				last_req.kind = SET_LED;
				last_req.len = rq->wLength.word;
			}
            return USB_NO_MSG; // to get data in usbFunctionWrite
		} else
			DBG1(0x51,0,0);
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

uchar usbFunctionWrite(uchar *data, uchar len)
{
	if (last_req.len == 0) {
		return -1;
	}
	switch (last_req.kind) {
		case SET_LED:
			vusb_keyboard_leds = data[0];
			last_req.len = 0;
			//DBG1(0x52,data,1);
			return 1;
			break;
		case NONE:
		default:
			return -1;
			break;
	}
	return 1;
}


/* ------------------------------------------------------------------------- */
extern volatile uint8_t ps2_error;
extern volatile uchar inTokenRec;
void timer0_init(void)
{
	
	// 1 millisecond interrupt
	OCR0A = 200; // przy prescaller = 1024 mamy co 10 ms interrupt.
	TIMSK0 = (1<<OCIE0A);
	TCNT0 = 0;
	inTokenRec = 0 ; //reset in count var
	// Timer1 CTC mode with clk/1024
	TCCR0A = ( 1 << WGM01 );
	TCCR0B = 5; // prescaller 1024
}
void timer0_disable(void){
	OCR0A = 0;
	TIMSK0 = 0;
	TCNT0 = 0;
	TCCR0A =0;
	TCCR0B=0;
}


void adjustKeyboardLeds(){
	uchar uncode = 0;
	if ( /* 0 &*/ (vusb_keyboard_leds != keyboard_leds  )){
		// check which led is different
		keyboard_leds ^= vusb_keyboard_leds;
		
		if (keyboard_leds &  (1<<USB_LED_SCROLL_LOCK)){
			// unregister key is scroll lock KC_SCROLLLOCK
			uncode = KC_SCROLLLOCK;
		} else if ( keyboard_leds &  (1<<USB_LED_NUM_LOCK)) {
			uncode = KC_NUMLOCK;
		} else if ( keyboard_leds &  (1<<USB_LED_CAPS_LOCK)) {
			uncode = KC_CAPSLOCK;
		}			
		
		keyboard_leds = vusb_keyboard_leds;
		uint8_t ps2_led = 0;
		if (keyboard_leds &  (1<<USB_LED_SCROLL_LOCK))
			ps2_led |= (1<<PS2_LED_SCROLL_LOCK);
		if (keyboard_leds &  (1<<USB_LED_NUM_LOCK))
			ps2_led |= (1<<PS2_LED_NUM_LOCK);
		if (keyboard_leds &  (1<<USB_LED_CAPS_LOCK))
			ps2_led |= (1<<PS2_LED_CAPS_LOCK);
		timer0_disable();
		ps2_host_set_led(ps2_led);
		unregister_code(uncode);
		// TODO: we could turn on timer only if we are in sleep mode to reduce int requests
		timer0_init(); 
		//DBG2(0x67,0,0);
	}
}

void WDT_off(void)
{
	cli();
	wdt_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x00;
	//sei();
}

void pcint_init(void){
	DDRC |= 1; // change to output
	PCICR |= 2; // enable pcint1
	PCMSK1 |= 1; // enable pc0 pin change
}
uint16_t timer0_count = 0 ;
uint8_t suspendMode = 0;

ISR(TIMER0_COMPA_vect)
{
	timer0_count++;
	sei();
	if (timer0_count == 5  && suspendMode )
		PORTC |= 2;
	if (timer0_count > 100){
		timer0_count = 0;
		DBG2(0x0c, &inTokenRec, 1);
		if (inTokenRec == 0 )
			suspendMode = 1;
		else
			suspendMode = 0;
		if (suspendMode)
			PORTC &= ~2;
		inTokenRec = 0;
	}
	return;
}

/*
	program entry
*/

int __attribute__((noreturn)) main(void)
{
	uchar   i;
	suspendMode = 0;
	WDT_off();
    //wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
#if DEBUG_LEVEL > 0
	odDebugInit(0);	// prog usart
#endif
	pcint_init();
	PORTC |= 2; // set portc1 bit to 1
	
   
	ps2_host_init();
	ps2_mouse_init();
    DBG1(0x00, 0, 0);       /* debug output: main starts */
	//timer_init();
	timer0_init();
	DDRC |= 2; // set PORTC1 to output
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
	//sei();
	uint8_t readTimer = 1;
	uint16_t timerVal = 0;
	uint8_t leds = 7;
	uint16_t ela = timer_read();
	matrix_init();
	while ( 0 ){
		_delay_us(100);
		//wdt_reset();
		//matrix_scan();			
		//if (timer_elapsed(ela) > 1000 ){
		//	ela = timer_read();
		//	ps2_host_set_led(leds);
		//	leds ^= 7;
			
			//leds = 0 ;
			//DBG2(0x56,&numEnt,1);
		//}			
	}
	
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
	
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */
    for(;;){                /* main event loop */
        
       // wdt_reset();
        usbPoll();
        //if(usbInterruptIsReady3()){
            ///* called after every poll of the interrupt endpoint */
            //advanceCircleByFixedAngle();
            //DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
			//send_mouse(&reportBuffer);
            ////usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        //}
		// wait 3 sec for init
		//if (timer_elapsed(startup)  > 3000){
			//startupDone = 1;
			//ela = timer_read();
		//}
					//
					//DBG1(0x02, 0, 0);
		if (usbConfiguration){
			//i--;
			//if (i == 0 && timer_elapsed(ela) >= 1000 ){
				//ela = timer_read();
				//register_code(KC_A);
				//unregister_code(KC_A);
			//}
			adjustKeyboardLeds();
			matrix_scan();
			mouse_task();
			vusb_transfer_keyboard();
			vusb_transfer_mouse();
			
		}		
	}	
}

/* ------------------------------------------------------------------------- */
