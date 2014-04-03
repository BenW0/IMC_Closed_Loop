/* Common Header */

#ifndef __common_h_
#define __common_h_

#include <mk20dx128.h>
#include <stdint.h>
#include <stdbool.h>

// Min and Max functions:
//FORCE_INLINE uint8_t max8(uint8_t a, uint8_t b) {return a > b ? a : b;}
// We need a max macro that doesn't execute x or y multiple times.
#define max(x, y)     ({typeof(x) _x = (x); typeof(y) _y = (y); _x > _y ? _x : _y;})
#define min(x, y)     ({typeof(x) _x = (x); typeof(y) _y = (y); _x < _y ? _x : _y;})

#endif