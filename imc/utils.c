#include "utils.h"
#include <stdint.h>

void vmemset(volatile void *ptr, uint8_t val, uint32_t size){
  volatile uint8_t* dst = (uint8_t*) ptr;
  if(!dst) return;
  while(size-->0){
    *dst++ = val;
  }
}

void vmemcpy(void* dest, volatile void* source, uint32_t size){
  volatile uint8_t* dst = (uint8_t*) dest;
  volatile uint8_t* src = (uint8_t*) source;
  if(!dst || !src) return;
  while(size-->0){
    *dst++ = *src++;
  }
}
