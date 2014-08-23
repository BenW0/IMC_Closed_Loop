/* Common Header.

 * Code credit for random number generation: http://stackoverflow.com/questions/1167253/implementation-of-rand
 
 * This software is (c) 2014 by Ben Weiss and is released under the following license:
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Ben Weiss
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __common_h_
#define __common_h_

// Make Visual Studio happy...
#ifdef _MSC_VER
#define __attribute__(x)
#define __inline__
#define __cplusplus
#define __builtin_va_list int
#define __asm__(x)
#define F_CPU  48000000
#define F_BUS 48000000
#define __MK20DX256__
#define USB_RAWHID
#endif 

#include <usb_desc.h>
#include <usb_seremu.h>

#include <mk20dx128.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include <usb_rawhid.h>
#include <stdio.h>
#include <string.h>

#include "rawhid_msg.h"

// This define controls whether to use the QD-input or the SPI-input encoder. Disable
// to use the SPI-based encoder
//#define USE_QD_ENC

// datatype used for control functions (for future conversion to double?)
typedef float real;

// In main.c:main(), I am co-opting the Teensy's systick_millis_count functionality to increase
// the amount of time between rollovers of the SYST counter. This function computes the current
// time in 10us increments. NOTE: I think it would be a significant time-savings to switch the time base from
// ms to some even power of two off of the systic counter and just do all our timing in those units.
// Rollover is every ~12 hours.
// To implement this in other projects, include:
// SYST_RVR = (F_CPU / 1000) * SYSTICK_UPDATE_MS - 1; //in main
// and add the systick_isr at the bottom of main. Note the builtin delay() stops working; use
// delay_real() below. Because imc needs delay() and usb_rawhid.c uses systick_millis_count, 
// I have reverted the system to use 1ms updates again.
// If it is having issues with not correctly reporting just when the systick counter is incremented
// (i.e. get_systic_tenus returns, successively, 97, 98, 99, 0, 100 because systick_millis_count has not been
// incremented by the systic isr even though CVR has reset), consider using this code:
  //__disable_irq();
  //uint32_t cvr = SYST_CVR;
  //uint32_t cnt = (SYST_RVR - SYST_CVR) * 1000L / (F_CPU / 100L) + // 10*us on the timer now
  //                systick_millis_count * SYSTICK_UPDATE_TEN_US;   // accumulated 10*us      was systick_tenus_count
  //__enable_irq();
  //if(SYST_RVR - cvr < 4)    // counter rollover has occcurred but not been captured by systick_millis_count
  //  return cnt + SYSTICK_UPDATE_TEN_US;
  //else
  //  return cnt;
// I discovered the hard way that reversing the lines of the cnt = (so we read systick_millis_count before SYST_CVR)
// results in occasional errors of this kind, but so far the code below seems to be working.
#define SYSTICK_UPDATE_TEN_US      100     // Frequency of update for systic timer (default Teensy is 1ms=100)
extern volatile uint32_t systick_tenus_count;     // millisecond counter which updates every SYSTICK_UPDATE_MS ms.
extern volatile uint32_t systick_millis_count;
__attribute__ ((always_inline)) inline uint32_t get_systick_tenus(void)
{
  return (SYST_RVR - SYST_CVR) * 1000L / (F_CPU / 100L) + // 10*us on the timer now
                  systick_millis_count * SYSTICK_UPDATE_TEN_US;   // accumulated 10*us      was systick_tenus_count
}
// The above hack causes delay() as implemented in util.c to fail. Instead, we'll use one that depends on
// our enhanced counter. Note that it's accuracy is somewhat reduced due to computations required for 
//the ms counter, probably still accurate to within 1us or so.
__attribute__ ((always_inline)) inline void delay_real(uint32_t ms)
{
  uint32_t endtick = get_systick_tenus() + ms * 100L;
  // counter rollover?
  while(get_systick_tenus() > endtick)
    ;
  while(get_systick_tenus() < endtick)
    ;
} 

// Sometimes for bitbanging a bus I need finer delay control than delay_microseconds. Below is a function
// wich delays only 1/8 (0.125) us per tic
static inline void delay_0125us(uint32_t) __attribute__((always_inline, unused));
static inline void delay_0125us(uint32_t tics)
{
#if F_CPU == 96000000
	uint32_t n = tics << 2;
#elif F_CPU == 48000000
	uint32_t n = tics << 1;
#elif F_CPU == 24000000
	uint32_t n = tics;
#endif
	if (tics == 0) return;
	asm volatile(
		"L_%=_delay_microseconds:"		"\n\t"
		"subs   %0, #1"				"\n\t"
		"bne    L_%=_delay_microseconds"		"\n"
		: "+r" (n) :
	);
}

// a few time-related constants
#define TENUS_PER_MIN_F       ((float)6.e6)
#define TENUS_PER_MIN_I       6000000L
#define TENUS_PER_SEC_F       ((float)1.e5)
#define TENUS_PER_SEC_I       100000L
#define MIN_PER_TENUS_F       ((float)1.6666666666667e-7)


// Random number generator: (lfsr113)
uint32_t rand_uint32 (void);


// The Teensy development tools don't give good access to allow masking of interrupt priorities
// (effectively elevating the current level of execution to a more urgent (=lower #) level)
// The following follows ARM DDI 0403D B5-805, with hints from mk20dx128.c:nvic_execution_priority.
// The parameter pri should be of the same form used by mk20dx128.h:NVIC_SET_PRIORITY()
#define SET_BASEPRI(pri)    asm volatile("msr basepri, %0\n" :: "r" ((pri)) : );
#define CLEAR_BASEPRI()     asm volatile("movs r0, #0\n\
                                          msr basepri, r0" ::: "r0");

// Min and Max functions:
//FORCE_INLINE uint8_t max8(uint8_t a, uint8_t b) {return a > b ? a : b;}
// We need a max macro that doesn't execute x or y multiple times.
#define max(x, y)     ({typeof(x) _x = (x); typeof(y) _y = (y); _x > _y ? _x : _y;})
#define min(x, y)     ({typeof(x) _x = (x); typeof(y) _y = (y); _x < _y ? _x : _y;})


// Lerp helper function
#define lerp(a, b, t)   ((a) + ((b) - (a)) * (t))



#define PI 3.14159265f

#endif