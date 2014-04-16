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
//#include "SPIFIFO.h"


#ifndef USE_QD_ENC
 
// Global Variables ==========================================

 
// Local Variables ===========================================
static volatile long encoder_offset = 0;   // added to the actual encoder value when read.
static char message[100] = "Hello, World";
 
// enc_init()
// Initializes the SPI interface and the device as required.
void enc_Init(void)
{
	
}


// get_qenc_value()
// returns the current encoder tic index
int32_t get_spienc_value(void)
{
	return 0; //||\\!!
}

// set_qenc_value()
// sets the current encoder tic index (by offsetting the current value)
void set_spienc_value(int32_t newvalue)
{

}

#endif