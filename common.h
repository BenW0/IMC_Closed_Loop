/* Common Header */

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
#endif 

#include <mk20dx128.h>
#include <stdint.h>
#include <stdbool.h>

#include <usb_serial.h>
#include <stdio.h>
#include <string.h>

// This define controls whether to use the QD-input or the SPI-input encoder. Disable
// to use the SPI-based encoder
#define USE_QD_ENC

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
// delay_real() below.
#define SYSTICK_UPDATE_TEN_US      1000     // Frequency of update for systic timer (default Teensy is 1ms)
extern volatile uint32_t systick_tenus_count;     // millisecond counter which updates every SYSTICK_UPDATE_MS ms.
__attribute__ ((always_inline)) inline uint32_t get_systick_tenus(void)
{
  uint32_t val = systick_tenus_count +   // accumulated 10*us
    (SYST_RVR - SYST_CVR) * 1000L / (F_CPU / 100L);  // 10*us on the timer now
  return val;
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

// Min and Max functions:
//FORCE_INLINE uint8_t max8(uint8_t a, uint8_t b) {return a > b ? a : b;}
// We need a max macro that doesn't execute x or y multiple times.
#define max(x, y)     ({typeof(x) _x = (x); typeof(y) _y = (y); _x > _y ? _x : _y;})
#define min(x, y)     ({typeof(x) _x = (x); typeof(y) _y = (y); _x < _y ? _x : _y;})


// Lerp helper function
#define lerp(a, b, t)   ((a) + ((b) - (a)) * (t))



#define PI 3.14159265f

#endif