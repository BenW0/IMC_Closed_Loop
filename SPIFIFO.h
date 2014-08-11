// This file is modified from the Teensyduino codebase, converted for a C environment, with some of the 
// compatibility fluff removed.

#ifndef _SPIFIFO_h_
#define _SPIFIFO_h_

#include <core_pins.h>
#include <mk20dx128.h>
#include <stdint.h>
#include <stdbool.h>



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


inline __attribute__((always_inline)) void spififo_clear(void)  {
	SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
}

inline __attribute__((always_inline)) void spififo_begin(uint8_t pin, uint32_t speed, uint32_t mode)  {
	uint32_t p, ctar = speed;
	SIM_SCGC6 |= SIM_SCGC6_SPI0;

	SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F);    // see mk20dx128.h:1370 for this clever definition.
	if (mode & 0x08) ctar |= SPI_CTAR_CPOL;
	if (mode & 0x04) {
		ctar |= SPI_CTAR_CPHA;
		ctar |= (ctar & 0x0F) << 8;
	} else {
		ctar |= (ctar & 0x0F) << 12;
	}
	SPI0.CTAR0 = ctar | SPI_CTAR_FMSZ(7);
	SPI0.CTAR1 = ctar | SPI_CTAR_FMSZ(15);
	if (pin == 10) {         // PTC4
		CORE_PIN10_CONFIG = PORT_PCR_MUX(2);
		p = 0x01;
	} else if (pin == 2) {   // PTD0
		CORE_PIN2_CONFIG = PORT_PCR_MUX(2);
		p = 0x01;
	} else if (pin == 9) {   // PTC3
		CORE_PIN9_CONFIG = PORT_PCR_MUX(2);
		p = 0x02;
	} else if (pin == 6) {   // PTD4
		CORE_PIN6_CONFIG = PORT_PCR_MUX(2);
		p = 0x02;
	} else if (pin == 20) {  // PTD5
		CORE_PIN20_CONFIG = PORT_PCR_MUX(2);
		p = 0x04;
	} else if (pin == 23) {  // PTC2
		CORE_PIN23_CONFIG = PORT_PCR_MUX(2);
		p = 0x04;
	} else if (pin == 21) {  // PTD6
		CORE_PIN21_CONFIG = PORT_PCR_MUX(2);
		p = 0x08;
	} else if (pin == 22) {  // PTC1
		CORE_PIN22_CONFIG = PORT_PCR_MUX(2);
		p = 0x08;
	} else if (pin == 15) {  // PTC0
		CORE_PIN15_CONFIG = PORT_PCR_MUX(2);
		p = 0x10;
	} else {
		// not available
		//reg = portOutputRegister(pin);
		//*reg = 1;
		//pinMode(pin, OUTPUT);
		//p = 0;
	}
	pcs = p;
	spififo_clear();
	//SPCR.enable_pins(); -- inlined below for the default SPI port (11,12,13)
	CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // DOUT/MOSI = 11 (PTC6)
	CORE_PIN12_CONFIG = PORT_PCR_MUX(2);  // DIN/MISO = 12 (PTC7)
	CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // SCK = 13 (PTC5)
}

inline __attribute__((always_inline)) void spififo_write(uint32_t b, uint32_t cont) {
	uint32_t pcsbits = pcs << 16;
	if (pcsbits) {
		SPI0.PUSHR = (b & 0xFF) | pcsbits | (cont ? SPI_PUSHR_CONT : 0);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	} else {
		*reg = 0;
		SPI0.SR = SPI_SR_EOQF;
		SPI0.PUSHR = (b & 0xFF) | (cont ? 0 : SPI_PUSHR_EOQ);
		if (cont) {
			while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ;
		} else {
			while (!(SPI0.SR & SPI_SR_EOQF)) ;
			*reg = 1;
		}
	}
}

inline __attribute__((always_inline)) void spififo_write16(uint32_t b, uint32_t cont) {
	uint32_t pcsbits = pcs << 16;
	if (pcsbits) {
		SPI0.PUSHR = (b & 0xFFFF) | (pcs << 16) |
			(cont ? SPI_PUSHR_CONT : 0) | SPI_PUSHR_CTAS(1);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ;
	} else {
		*reg = 0;
		SPI0.SR = SPI_SR_EOQF;
		SPI0.PUSHR = (b & 0xFFFF) | (cont ? 0 : SPI_PUSHR_EOQ) | SPI_PUSHR_CTAS(1);
		if (cont) {
			while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ;
		} else {
			while (!(SPI0.SR & SPI_SR_EOQF)) ;
			*reg = 1;
		}
	}
}

inline __attribute__((always_inline)) uint32_t spififo_read(void) {
	while ((SPI0.SR & (15 << 4)) == 0) ;  // TODO, could wait forever
	return SPI0.POPR;
}



#endif
#endif
