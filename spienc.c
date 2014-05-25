/********************************************************************************
 * SPI Encoder Decode File
 * Ben Weiss, University of Washington 2014
 * Purpose: Reads an encoder's signal using the SPI interface
 * 
 * Source: 
 * 
 * 
 * License: None. This is for internal development only!
 ********************************************************************************/

#include "common.h"
#include <usb_serial.h>
#include <string.h>
#include <stdio.h>

#include "spienc.h"


#ifndef USE_QD_ENC

#include "SPIFIFO.h"
 
// Constants =================================================
const uint32_t ROLLOVER = 4096;                 // rollover of the internal SPI counter
const uint32_t MS_BEFORE_LOST_TRACK = 6;       // If we don't get an update at least this often, we may lose track of the counter rollover (2 mm / 400 mm/sec)

// Global Variables ==========================================

 
// Local Variables ===========================================
static volatile long encoder_offset = 0;   // added to the actual encoder value when read.
static char message[100] = "Hello, World";
static uint32_t last_update_millis = 0, readval, last_readval = 0;
static int32_t rollovers = 0;
static bool lost_track = true;
static int32_t offset = 0;

// Local Function Defines ====================================
uint8_t read_spi(uint32_t *value);
 

// enc_init()
// Initializes the SPI interface and the device as required.
void enc_Init(void)
{
  spififo_begin(10, SPI_CLOCK_1MHz, SPI_MODE2);
  last_update_millis = get_systick_tenus() * 100;

  if(read_spi(&readval))
  {
		last_readval = readval;
    lost_track = false;
  }
}

// enc_idle()
// idle task for the encoder. We use this to keep track of rollovers
void enc_idle(void)
{
  if(!read_spi(&readval))
		{
			// check for rollover
			if(readval > ROLLOVER * 3 / 4 && last_readval < ROLLOVER / 4)
			{
				// rolled over negative
				rollovers--;
			}
			if(readval < ROLLOVER / 4 && last_readval > ROLLOVER * 3 / 4)
			{
				// rolled over positive
				rollovers++;
			}
			last_readval = readval;
		}
  else
  {
    // something's wrong! Assume we lost track
    lost_track = true;
  }
  // has it been too long since we had an update?
  if(get_systick_tenus() - last_update_millis > MS_BEFORE_LOST_TRACK)
    lost_track = true;
  last_update_millis = get_systick_tenus() * 100;
}


// get_enc_value()
// returns the current encoder tic index
uint8_t get_enc_value(volatile int32_t *value)
{
	*value = rollovers * ROLLOVER + (int32_t)readval + offset;
  return lost_track ? 1 : 0;
}

// set_enc_value()
// sets the current encoder tic index (by offsetting the current value)
void set_enc_value(int32_t newvalue)
{
  offset = newvalue - rollovers * ROLLOVER + (int32_t)readval;
  lost_track = false;
}


// reads the encoder value over SPI. Returns 0 if read was successful, 1 otherwise.
uint8_t read_spi(uint32_t *value)
{
	uint8_t i;
	// write out some uint8_ts...The content is bogus, we just need the clock to fire.
	spififo_write16(0x1234, 1);
	spififo_write16(0x1234, 0);
	uint32_t inp = spififo_read();
	uint32_t inp2 = spififo_read();
	inp = (inp << 16) | inp2;
	// first bit (msb) is garbage
	inp &= 0x7fffffff;
	// next 12 bits are the reading
	uint32_t rd = (inp >> 19);
	// next 5 bits are flags
	uint8_t ocf = (inp & (1 << 18)) ? 1 : 0;
	uint8_t cof = (inp & (1 << 17)) ? 1 : 0;
	uint8_t lin = (inp & (1 << 16)) ? 1 : 0;
	uint8_t mag_inc = (inp & (1 << 15)) ? 1 : 0;
	uint8_t mag_dec = (inp & (1 << 14)) ? 1 : 0;
	uint8_t par = (inp & (1 << 13)) ? 1 : 0;
	// check parity bit
	uint8_t parcheck = 0;
	for(i = 14; i < 31; i++)
		if(inp & (1 << i)) parcheck ^= 1;
	// valid packet if ocf = 1, cof = 0, lin = 0, mag_inc = 0, mag_dec = 0, parcheck == par.
	if(!ocf || cof || lin || mag_inc || mag_dec || parcheck != par)
		return 1;
	*value = rd;
	return 0;
}

#endif