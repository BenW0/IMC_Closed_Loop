#ifndef utils_h
#define utils_h

#include <stdint.h>
void vmemset(volatile void *,uint8_t,uint32_t
);
// memcopy from a volatile dest
void vmemcpy(void*, volatile void*, uint32_t);
#endif
