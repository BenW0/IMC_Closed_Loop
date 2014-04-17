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

// Min and Max functions:
//FORCE_INLINE uint8_t max8(uint8_t a, uint8_t b) {return a > b ? a : b;}
// We need a max macro that doesn't execute x or y multiple times.
#define max(x, y)     ({typeof(x) _x = (x); typeof(y) _y = (y); _x > _y ? _x : _y;})
#define min(x, y)     ({typeof(x) _x = (x); typeof(y) _y = (y); _x < _y ? _x : _y;})

// This define controls whether to use the QD-input or the SPI-input encoder. Disable
// to use the SPI-based encoder
//#define USE_QD_ENC

#endif