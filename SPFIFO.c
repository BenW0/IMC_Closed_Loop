// This file implements a conversion of the Teensyduino SPIFIFO.h for a C environment
// without needing the overhead of the Teensyduino environment.

// This file is modified from the Teensyduino codebase, converted for a C environment, with some of the 
// compatibility fluff removed.

#ifndef _SPIFIFO_h_
#define _SPIFIFO_h_

#include "common.h"
#include "SPIFIFO.h"
#include "td-inc/core_pins.h"



// Local Variables ================================================
static uint8_t pcs = 0;
static volatile uint8_t *reg = 0;


inline void begin(uint8_t pin, uint32_t speed, uint32_t mode) __attribute__((always_inline)) {
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
		reg = portOutputRegister(pin);
		*reg = 1;
		pinMode(pin, OUTPUT);
		p = 0;
	}
	pcs = p;
	clear();
	SPCR.enable_pins();
}

inline void write(uint32_t b, uint32_t cont) __attribute__((always_inline)) {
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

inline void write16(uint32_t b, uint32_t cont) __attribute__((always_inline)) {
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

inline uint32_t read(void) __attribute__((always_inline)) {
	while ((SPI0.SR & (15 << 4)) == 0) ;  // TODO, could wait forever
	return SPI0.POPR;
}

inline void clear(void) __attribute__((always_inline)) {
	SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
}
