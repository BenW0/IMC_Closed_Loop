
#include "QEI.h"

extern uint32 __VECTOR_RAM[];

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
    PORTB_PCR18 = PORT_PCR_MUX(6) | PORT_PCR_PFE_MASK; // FTM1 CH0
    PORTB_PCR19 = PORT_PCR_MUX(6) | PORT_PCR_PFE_MASK; // FTM1 CH1

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
    enable_irq(INT_FTM2-16);
    enable_irq(INT_CMP1-16);
    /***********************/

}

void QEI_Deinit(void)
{
	/* disable interrupts */
    disable_irq(INT_FTM2-16);
    disable_irq(INT_CMP1-16);	
	/* FTM module disable */
    SIM_SCGC3 &= ~(SIM_SCGC3_FTM2_MASK);
    SIM_SCGC4 &= ~(SIM_SCGC4_CMP_MASK);
}


/********************************************************************************
 *   QEI_isr: Quadrature encode interrupt subroutine
 * Notes:
 *    -
 ********************************************************************************/

void QEI_isr(void)
{ static uint16 Index_Last = 0x8000;
  uint16 C0V, C1V;
  int16 Diff;
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
       printf("       %04x %04x  UP  %6d\n",C0V,C1V,Diff);
     else
       printf("       %04x %04x  DN  %6d\n",C0V,C1V,Diff);
     FTM2_C0SC = /*FTM_CnSC_CHIE_MASK | */FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK;
     FTM2_C1SC = FTM_CnSC_CHIE_MASK | FTM_CnSC_MSA_MASK | FTM_CnSC_ELSB_MASK;
  }//We may want to disable this interrupt for a while from here, for something less than any possible 'true' period
      // to avoid multiple edge hits
  if( FTM2_SC & FTM_SC_TOF_MASK )
  {
     FTM2_SC = FTM_SC_TOIE_MASK | FTM_SC_CLKS(3);
     if( FTM2_QDCTRL & FTM_QDCTRL_TOFDIR_MASK )
       printf("Overflow UP\n");
     else
       printf("Overflow DOWN\n");
  }//We can also add a counter to this to extend the encoder count, BUT edge conditions could read a false pairing
}

/********************************************************************************
 *   CPM1_isr: Compare 1 interrupt subroutine
 * Notes:     Come here on BOTH edges of the Index pulse, sort 'em out later
 *    -
 ********************************************************************************/

void CPM1_isr(void)
{
  if( CMP1_SCR & CMP_SCR_CFR_MASK )
    printf("%05x rise\n",FTM2_CNT);
  else
    printf("%05x fall\n",FTM2_CNT);
  CMP1_SCR = CMP_SCR_IER_MASK | CMP_SCR_IEF_MASK | CMP_SCR_CFR_MASK | CMP_SCR_CFF_MASK; //Zero-out the Interupt bits
}