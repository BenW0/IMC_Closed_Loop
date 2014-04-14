/********************************************************************************
 * Quadrature Encoder Decode File
 * Ben Weiss, University of Washington 2014
 * Purpose: Decodes a quadrature encoder's signal using the FTM2 and CMP1 modules
 * 
 * Source: This code is derived almost exactly from code posted to the Freescale development forum:
 *   https://community.freescale.com/thread/304806
 * I think, since I don't have an index pulse, that I can remove the CMP1 code and both interrupts
 *  but I don't want to do that until it works!
 * 
 * License: None. This is for internal development only!
 ********************************************************************************/

#include "common.h"
#include <usb_serial.h>
#include <string.h>
#include <stdio.h>

#include "qdenc.h"
 
// Global Variables ==========================================
volatile unsigned long isr1_count = 0, isr2_count = 0;
 
// Local Variables ===========================================
volatile long encoder_offset = 0;   // added to the actual encoder value when read.
static char message[100] = "Hello, World";
 
/********************************************************************************
 *   QEI_Init: Initializes FTM2 module for QD w/Index thru CMP1
 * Notes:
 *    -
 ********************************************************************************/
void QEI_Init(void)
{
    //enable the clock for FTM2 & CMP modules
    SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;
    SIM_SCGC4 |= SIM_SCGC4_CMP_MASK;

    //configuring the two FTM2 input pins to QD-input mode:
    PORTB_PCR18 = PORT_PCR_MUX(6) | PORT_PCR_PFE_MASK; // FTM2 CH0
    PORTB_PCR19 = PORT_PCR_MUX(6) | PORT_PCR_PFE_MASK; // FTM2 CH1

    //Set up, then enable the counter
    FTM2_SC = 0;
    FTM2_C0V = 0;
    FTM2_C1V = 0;        //Clear these while we can!
    FTM2_COMBINE = FTM_COMBINE_DECAPEN0_MASK;     //Prep for dual-edge capture
    FTM2_MODE = FTM_MODE_FTMEN_MASK;
    //enable the counter to run in the BDM mode
    FTM2_CONF = FTM_CONF_BDMMODE(3);
    //load the Modulo register and counter initial value
    FTM2_MOD = 0xFFFF;                       //Just maintain the full 16 bits
    FTM2_CNTIN = 0x8000;                     //Start with the counter 'centered' -- why not...
    FTM2_CNT = 0x8000;                       //The acutal load is from CNTIN not this value, but this is the result of the write
    FTM2_CNTIN = 0;                          //Further roll-overs run the rull range
    //configuring FTM for quadrature mode
    FTM2_QDCTRL = FTM_QDCTRL_PHAFLTREN_MASK  | FTM_QDCTRL_PHBFLTREN_MASK  |FTM_QDCTRL_QUADEN_MASK;
    // start the timer clock, source is the external (QD) clock
    FTM2_SC = FTM_SC_TOIE_MASK | FTM_SC_CLKS(3);     //And enable overflow interrupts for extended counts


    PORTC_PCR2 = PORT_PCR_MUX(0);       // CMP1 IN0 enable
    //Turn on comparator-DAC at half-VDD
    CMP1_DACCR = CMP_DACCR_DACEN_MASK | CMP_DACCR_VRSEL_MASK | CMP_DACCR_VOSEL(0x20);
    CMP1_CR0 = CMP_CR0_FILTER_CNT(7) | CMP_CR0_HYSTCTR(3);          //Maximal Filtering
    CMP1_MUXCR = CMP_MUXCR_MSEL(7) | CMP_MUXCR_PSEL(0);             //DAC-vs-IN0
    CMP1_FPR = 255;                                                 //More maximal Filter
    CMP1_CR1 = CMP_CR1_EN_MASK;
    CMP1_SCR = CMP_SCR_CFR_MASK | CMP_SCR_CFF_MASK;
    CMP1_SCR |= CMP_SCR_IER_MASK | CMP_SCR_IEF_MASK;          //Both edges, since we don't know which direction we are turning

    //Hook FTM2 CH0 capture to run from this comparator output!
    SIM_SOPT4 |= SIM_SOPT4_FTM2CH0SRC(2);

    FTM2_FILTER = FTM_FILTER_CH1FVAL(15) | FTM_FILTER_CH0FVAL(15);
    FTM2_C0SC = /*FTM_CnSC_CHIE_MASK | */FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK;
                            //Input rising Capture on Channel 0, mode for Continuous dual-cap
    FTM2_C1SC = FTM_CnSC_CHIE_MASK | FTM_CnSC_MSA_MASK | FTM_CnSC_ELSB_MASK;
                            //Input falling Capture on Channel 1
   //   printf("       %04x %04x  Start\n",FTM2_C0V,FTM2_C1V);
    FTM2_COMBINE |= FTM_COMBINE_DECAP0_MASK;  //Make dual-cap GO!

    /* Init encoder interrupts */
    NVIC_ENABLE_IRQ(IRQ_FTM2);//
    NVIC_ENABLE_IRQ(IRQ_CMP1);//
		//||\\??Should this be disabled?
    //sei();
    /***********************/

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