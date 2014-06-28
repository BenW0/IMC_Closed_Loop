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
#include <pin_config.h>
#include <util.h>

#include "spienc.h"


#ifndef USE_QD_ENC

#include "SPIFIFO.h"
 
// Constants =================================================
const uint32_t ROLLOVER = 4096;                 // rollover of the internal SPI counter
//const uint32_t TENUS_BEFORE_LOST_TRACK = 600;       // If we don't get an update at least this often, we may lose track of the counter rollover (2 mm / 400 mm/sec)
#define DISP_BEFORE_LOST_TRACK    1500          // If the encoder value has changed by more than this, we might have skipped a step!

// Global Variables ==========================================

 
// Local Variables ===========================================
static volatile long encoder_offset = 0;   // added to the actual encoder value when read.
//static char message[100];
static uint32_t last_update_tenus = 0, readval, last_readval = 0;
static int32_t rollovers = 0;
static bool lost_track = true;
static int32_t offset = 0;

// Local Function Defines ====================================
void read_enc(void);
uint8_t read_spi(uint32_t *value);
uint32_t spibitbang_read(void);
 

// enc_init()
// Initializes the SPI interface and the device as required.
void enc_Init(void)
{
#ifdef ENC_USE_SPIFIFO
  spififo_begin(10, SPI_CLOCK_1MHz, SPI_MODE2);
#else
  // setup the pins
  ENC_SCK_CTRL = STANDARD_OUTPUT;
  ENC_SCK_PORT(DDR) |= ENC_SCK_BIT;
  SCK_ON();

  ENC_DIN_CTRL = MUX_GPIO;
  ENC_DIN_PORT(DDR) &= ~ENC_DIN_BIT;

  ENC_CS_CTRL = STANDARD_OUTPUT;
  ENC_CS_PORT(DDR) |= ENC_CS_BIT;
  CS_ON();
#endif    // !ENC_USE_SPFIFO

  last_update_tenus = get_systick_tenus();

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
  read_enc();
}

// this function reads the encoder and traps rollovers.
void read_enc(void)
{
  uint32_t time = get_systick_tenus();
  bool rolled = false;
  if(!read_spi(&readval))
		{
			// check for rollover
			if(readval > ROLLOVER * 3 / 4 && last_readval < ROLLOVER / 4)
			{
				// rolled over negative
				rollovers--;
        rolled = true;
			}
			else if(readval < ROLLOVER / 4 && last_readval > ROLLOVER * 3 / 4)
			{
				// rolled over positive
				rollovers++;
        rolled = true;
			}
      // check for high absolute change --> possibility of skipping a step.
      if(labs((rolled ? ROLLOVER : 0) - labs(last_readval - readval)) > DISP_BEFORE_LOST_TRACK)
      {
        if(!lost_track)
        {
          hid_printf("'High Enc Change: %u. readval = %u, last_readval = %u, Time change = %u\n", 
            (unsigned int)labs((rolled ? -ROLLOVER : 0) + labs(last_readval - readval)), 
            (unsigned int)readval, 
            (unsigned int)last_readval, 
            (unsigned int)(time - last_update_tenus));
        }
        lost_track = true;
      }

			last_readval = readval;
		}
  else
  {
    // something's wrong! Assume we lost track
    lost_track = true;
  }
  last_update_tenus = time;
}


// get_enc_value()
// returns the current encoder tic index
uint8_t get_enc_value(volatile int32_t *value)
{
  read_enc();
	*value = rollovers * ROLLOVER + (int32_t)readval + offset;
  return lost_track ? 1 : 0;
}

// set_enc_value()
// sets the current encoder tic index (by offsetting the current value)
void set_enc_value(int32_t newvalue)
{
  offset = newvalue - rollovers * ROLLOVER - (int32_t)readval;
  lost_track = false;
}


// reads the encoder value over SPI. Returns 0 if read was successful, 1 otherwise.
uint8_t read_spi(uint32_t *value)
{
	uint8_t i;
	// write out some uint8_ts...The content is bogus, we just need the clock to fire.
  // we need to block the control interrupt from firing while we read the serial port (if it fires half way through
  // reading, we'll be in big trouble!)
  SET_BASEPRI(2<<4);
#ifdef ENC_USE_SPIFIFO
	spififo_write16(0x1234, 1);
	spififo_write16(0x1234, 0);
	uint32_t inp = spififo_read();
	uint32_t inp2 = spififo_read();
	inp = (inp << 16) | inp2;
#else
  uint32_t inp;
  inp = spibitbang_read();
#endif
  CLEAR_BASEPRI();

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
	// valid packet if ocf = 1, cof = 0, lin = 0, mag_inc and mag_dec not both 1, parcheck == par.
	if(!ocf || cof || lin || (mag_inc && mag_dec) || parcheck != par)
  {
    //||\\!!hid_printf("'Lost Track: ocf=%i cof=%i lin=%i mag_inc=%i mag_dec=%i parcheck=%i\n", ocf, cof, lin, mag_inc, mag_dec, (int)(parcheck != par));
    //usb_serial_write(message,strlen(message));
		return 1;
  }
	*value = rd;
	return 0;
}

// This routine bitbangs pins (see spienc.h for pinnout) to read the encoder. This is NOT a full implementation of 
// software SPI, but just enough to do the job for the chip at hand. As a result, message length, etc. is hard coded.
uint32_t spibitbang_read()
{
  uint32_t data = 0;
  // pull down CS to signal the start of a transfer. SCK should have been left high after the last round.
  CS_OFF();
  delay_microseconds(1);    // >600 ns

  for(uint8_t i = 0; i < 18; ++i)
  {
    SCK_OFF();
    // Take the reading
    data |= (DIN_READ() << (31 - i));
    delay_0125us(4);    // 500 ns
    SCK_ON();
    delay_0125us(4);    // 500 ns
  }
  // read one last bit (parity) without cycling the clock back down
  data |= (DIN_READ() << 12);   // (31 - 19)
  // signal end of read...
  CS_ON();

  return data;
}

#endif