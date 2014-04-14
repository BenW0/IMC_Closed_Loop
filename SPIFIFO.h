// This file is modified from the Teensyduino codebase, converted for a C environment, with some of the 
// compatibility fluff removed.

#ifndef _SPIFIFO_h_
#define _SPIFIFO_h_



// sck = F_BUS / PBR * ((1+DBR)/BR)
//  PBR = 2, 3, 5, 7
//  DBR = 0, 1         -- zero preferred
//  BR = 2, 4, 6, 8, 16, 32, 64, 128, 256, 512


#if F_BUS == 48000000

#define HAS_SPIFIFO
#define SPI_CLOCK_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(48 / 2) * ((1+1)/2)
#define SPI_CLOCK_16MHz   (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(48 / 3) * ((1+1)/2)  33% duty cycle
#define SPI_CLOCK_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0)) //(48 / 2) * ((1+0)/2)
#define SPI_CLOCK_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0)) //(48 / 3) * ((1+0)/2)
#define SPI_CLOCK_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(1)) //(48 / 2) * ((1+0)/4)
#define SPI_CLOCK_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(1)) //(48 / 3) * ((1+0)/4)
#define SPI_CLOCK_1MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(4)) //(48 / 3) * ((1+0)/32)
#define SPI_CLOCK_500KHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(5)) //(48 / 3) * ((1+0)/64)

#elif F_BUS == 24000000

#define HAS_SPIFIFO
#define SPI_CLOCK_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(24 / 2) * ((1+1)/2)  12 MHz
#define SPI_CLOCK_16MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(24 / 2) * ((1+1)/2)  12 MHz
#define SPI_CLOCK_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(24 / 2) * ((1+1)/2)
#define SPI_CLOCK_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(24 / 3) * ((1+1)/2)  33% duty cycle
#define SPI_CLOCK_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0)) //(24 / 2) * ((1+0)/2)
#define SPI_CLOCK_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0)) //(24 / 3) * ((1+0)/2)

#endif


#ifdef HAS_SPIFIFO

#ifndef SPI_MODE0
#define SPI_MODE0 0x00  // CPOL = 0, CPHA = 0
#define SPI_MODE1 0x04  // CPOL = 0, CPHA = 1
#define SPI_MODE2 0x08  // CPOL = 1, CPHA = 0
#define SPI_MODE3 0x0C  // CPOL = 1, CPHA = 1
#endif

#define SPI_CONTINUE 1

// Local Variables ================================================
static uint8_t pcs = 0;
static volatile uint8_t *reg = 0;


inline void begin(uint8_t pin, uint32_t speed, uint32_t mode) __attribute__((always_inline));

inline void write(uint32_t b, uint32_t cont) __attribute__((always_inline));
inline void write16(uint32_t b, uint32_t cont) __attribute__((always_inline));
inline uint32_t read(void) __attribute__((always_inline));
inline void clear(void) __attribute__((always_inline));


#endif
#endif
