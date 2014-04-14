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
#include "SPIFIFO.h"
 
// Global Variables ==========================================

 
// Local Variables ===========================================
volatile long encoder_offset = 0;   // added to the actual encoder value when read.
static char message[100] = "Hello, World";
 
// spi_init()
// Initializes the SPI interface and the device as required.
void spi_init(void)
{
	
}

void QEI_Deinit(void)
{
	/* disable interrupts */
    NVIC_DISABLE_IRQ(IRQ_FTM2);
    NVIC_DISABLE_IRQ(IRQ_CMP1);	
	/* FTM module disable */
    SIM_SCGC3 &= ~(SIM_SCGC3_FTM2_MASK);
    SIM_SCGC4 &= ~(SIM_SCGC4_CMP_MASK);
}


/********************************************************************************
 *   QEI_isr: Quadrature encode interrupt subroutine
 * Notes:
 *    -
 ********************************************************************************/

void ftm2_isr(void)
{ static uint16_t Index_Last = 0x8000;
  uint16_t C0V, C1V;
  int16_t Diff;
	isr1_count++;
  usb_serial_write("QEI_isr", 7);
  if( FTM2_C1SC & FTM_CnSC_CHF_MASK )
  {
     C0V = FTM2_C0V;   //Read to satisfy the dual-channel read-consistency logic
     C1V = FTM2_C1V;
     if( C0V != C1V )
     {
       if( C0V < C1V )
       {
         Diff = C0V - Index_Last;
         Index_Last = C0V;
       }else
       {
         Diff = C1V - Index_Last;
         Index_Last = C1V;
       }
     }else
       Diff = 0;
     if( FTM2_QDCTRL & FTM_QDCTRL_QUADIR_MASK )
     {
			sprintf(message, "%u, %u, %u UP\n", C0V, C1V, Diff);
			usb_serial_write(message,strlen(message));
     }
     else
     {
			sprintf(message, "%u, %u, %u DN\n", C0V, C1V, Diff);
			usb_serial_write(message,strlen(message));
     }
     FTM2_C0SC = /*FTM_CnSC_CHIE_MASK | */FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK;
     FTM2_C1SC = FTM_CnSC_CHIE_MASK | FTM_CnSC_MSA_MASK | FTM_CnSC_ELSB_MASK;
  }//We may want to disable this interrupt for a while from here, for something less than any possible 'true' period
      // to avoid multiple edge hits
  if( FTM2_SC & FTM_SC_TOF_MASK )
  {
     FTM2_SC = FTM_SC_TOIE_MASK | FTM_SC_CLKS(3);
     if( FTM2_QDCTRL & FTM_QDCTRL_TOFDIR_MASK )
       usb_serial_write("Overflow UP\n", 12);
     else
       usb_serial_write("Overflow DOWN\n", 14);
  }//We can also add a counter to this to extend the encoder count, BUT edge conditions could read a false pairing
}

/********************************************************************************
 *   CPM1_isr: Compare 1 interrupt subroutine
 * Notes:     Come here on BOTH edges of the Index pulse, sort 'em out later
 *    -
 ********************************************************************************/

void cpm1_isr(void)
{
	isr2_count++;
  if( CMP1_SCR & CMP_SCR_CFR_MASK )
  {
	sprintf(message, "%lu rise\n", FTM2_CNT);
    usb_serial_write(message,strlen(message));
  }
  else
  {
	sprintf(message, "%lu fall\n", FTM2_CNT);
    usb_serial_write(message,strlen(message));
  }
    
  CMP1_SCR = CMP_SCR_IER_MASK | CMP_SCR_IEF_MASK | CMP_SCR_CFR_MASK | CMP_SCR_CFF_MASK; //Zero-out the Interupt bits
}


// get_qenc_value()
// returns the current encoder tic index
int32_t get_qenc_value(void)
{
  return (int32_t)FTM2_CNT + encoder_offset;
}

// set_qenc_value()
// sets the current encoder tic index (by offsetting the current value)
void set_qenc_value(int32_t newvalue)
{
  encoder_offset = newvalue - (int32_t)FTM2_CNT;
}