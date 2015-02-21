/*
Copyright 2011 Jun Wako <wakojun@gmail.com>

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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "timer.h"


// counter resolution 1ms , 0.05 us
// NOTE: union { uint32_t timer32; struct { uint16_t dummy; uint16_t timer16; }}
volatile uint16_t timer_count = 0;

void timer_init(void)
{
    
	// 1 millisecond interrupt
    OCR1A = 20000;
    TIMSK1 = (1<<OCIE1A);
	TCNT1 = 0;
	
	// Timer1 CTC mode with no prescalling
	TCCR1B = ( 1 << WGM12 | 1<< CS10 );
}

inline
void timer_clear(void)
{
    uint8_t sreg = SREG;
    cli();
    timer_count = 0;
    SREG = sreg;
}

inline
uint16_t timer_read(void)
{
    uint16_t t;

    uint8_t sreg = SREG;
    cli();
    t = timer_count;
    SREG = sreg;

    return (t & 0xFFFF);
}

inline
uint16_t timer_read32(void)
{
    uint16_t t;

    uint8_t sreg = SREG;
    cli();
    t = timer_count;
    SREG = sreg;

    return t;
}

inline
uint16_t timer_elapsed(uint16_t last)
{
    uint16_t t;

    uint8_t sreg = SREG;
    cli();
    t = timer_count;
    SREG = sreg;

    return TIMER_DIFF_16((t & 0xFFFF), last);
}

inline
uint16_t timer_elapsed32(uint16_t last)
{
    uint16_t t;

    uint8_t sreg = SREG;
    cli();
    t = timer_count;
    SREG = sreg;

    return TIMER_DIFF_32(t, last);
}

// executed once per 1ms.(excess for just timer count?)
ISR(TIMER1_COMPA_vect)
{
    timer_count++;
	return;
}
