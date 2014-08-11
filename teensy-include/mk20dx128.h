/* Teensyduino Core Library (MODIFIED!)
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows 
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Modified by Ben Weiss to allow the inclusion of the (much more detailed) MK20DZ10.h 
 * register definition header from Freescale. The modification was done using the following
 * bash and awk commands:
 * awk -f awkscript -v ref=MK20DZ10.h mk20dx128.h > mk20dx128_new.h
 * cat awkscript
 * BEGIN {
 *   if (!ref) {
 *     printf "Usage: gawk -v ref=reffile -F " ARGV[0] " file-to-search\n";
 *     exit 1;
 *   }
 *   while (getline < ref) {
 * 		if ($1 == "#define") {
 * 			if (index($2, "(") == 0) {
 * 				words[$2] = "";
 * 			}
 * 			else {
 * 				split($2, key, "(");
 * 				words[key[1]] = "";
 * 			}
 * 		}
 *   }
 * }
 * 
 * {
 * 	 if($1 == "#define") {
 * 		 split($2, key, "(");
 * 		 if(!(key[1] in words)) print $0;
 * 	 }
 * 	 else { print $0;}
 * }

 */

#ifndef _mk20dx128_h_
#define _mk20dx128_h_

#include <MK20DZ10.h>

//#define F_CPU 96000000
//#define F_CPU 48000000
//#define F_CPU 24000000
//#define F_BUS 48000000
//#define F_BUS 24000000
//#define F_MEM 24000000

#if (F_CPU == 96000000)
 #define F_BUS 48000000
 #define F_MEM 24000000
#elif (F_CPU == 48000000)
 #define F_BUS 48000000
 #define F_MEM 24000000
#elif (F_CPU == 24000000)
 #define F_BUS 24000000
 #define F_MEM 24000000
#endif


#ifndef NULL
#define NULL ((void *)0)
#endif

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// chapter 11: Port control and interrupts (PORT)
#define PORT_PCR_ISF			(uint32_t)0x01000000		// Interrupt Status Flag
#define PORT_PCR_LK			(uint32_t)0x00008000		// Lock Register
#define PORT_PCR_DSE			(uint32_t)0x00000040		// Drive Strength Enable
#define PORT_PCR_ODE			(uint32_t)0x00000020		// Open Drain Enable
#define PORT_PCR_PFE			(uint32_t)0x00000010		// Passive Filter Enable
#define PORT_PCR_SRE			(uint32_t)0x00000004		// Slew Rate Enable
#define PORT_PCR_PE			(uint32_t)0x00000002		// Pull Enable
#define PORT_PCR_PS			(uint32_t)0x00000001		// Pull Select

// Chapter 12: System Integration Module (SIM)
#define SIM_SOPT1CFG		*(volatile uint32_t *)0x40047004 // SOPT1 Configuration Register
#define SIM_SOPT2_USBSRC		(uint32_t)0x00040000		// 0=USB_CLKIN, 1=FFL/PLL 
#define SIM_SOPT2_PLLFLLSEL		(uint32_t)0x00010000		// 0=FLL, 1=PLL
#define SIM_SOPT2_TRACECLKSEL		(uint32_t)0x00001000		// 0=MCGOUTCLK, 1=CPU
#define SIM_SOPT2_PTD7PAD		(uint32_t)0x00000800		// 0=normal, 1=double drive PTD7
#define SIM_SOPT2_CLKOUTSEL(n)		(uint32_t)(((n) & 7) << 5)	// Selects the clock to output on the CLKOUT pin.
#define SIM_SOPT2_RTCCLKOUTSEL		(uint32_t)0x00000010		// RTC clock out select
#define SIM_SCGC2_DAC0			(uint32_t)0x00001000		// DAC0 Clock Gate Control
#define SIM_SCGC3_ADC1			(uint32_t)0x08000000		// ADC1 Clock Gate Control
#define SIM_SCGC3_FTM2			(uint32_t)0x01000000		// FTM2 Clock Gate Control
#define SIM_SCGC4_VREF			(uint32_t)0x00100000		// VREF Clock Gate Control
#define SIM_SCGC4_CMP			(uint32_t)0x00080000		// Comparator Clock Gate Control
#define SIM_SCGC4_USBOTG		(uint32_t)0x00040000		// USB Clock Gate Control
#define SIM_SCGC4_UART2			(uint32_t)0x00001000		// UART2 Clock Gate Control
#define SIM_SCGC4_UART1			(uint32_t)0x00000800		// UART1 Clock Gate Control
#define SIM_SCGC4_UART0			(uint32_t)0x00000400		// UART0 Clock Gate Control
#define SIM_SCGC4_I2C1			(uint32_t)0x00000080		// I2C1 Clock Gate Control
#define SIM_SCGC4_I2C0			(uint32_t)0x00000040		// I2C0 Clock Gate Control
#define SIM_SCGC4_CMT			(uint32_t)0x00000004		// CMT Clock Gate Control
#define SIM_SCGC4_EWM			(uint32_t)0x00000002		// EWM Clock Gate Control
#define SIM_SCGC5_PORTE			(uint32_t)0x00002000		// Port E Clock Gate Control
#define SIM_SCGC5_PORTD			(uint32_t)0x00001000		// Port D Clock Gate Control
#define SIM_SCGC5_PORTC			(uint32_t)0x00000800		// Port C Clock Gate Control
#define SIM_SCGC5_PORTB			(uint32_t)0x00000400		// Port B Clock Gate Control
#define SIM_SCGC5_PORTA			(uint32_t)0x00000200		// Port A Clock Gate Control
#define SIM_SCGC5_TSI			(uint32_t)0x00000020		// Touch Sense Input TSI Clock Gate Control
#define SIM_SCGC5_LPTIMER		(uint32_t)0x00000001		// Low Power Timer Access Control
#define SIM_SCGC6_RTC			(uint32_t)0x20000000		// RTC Access
#define SIM_SCGC6_ADC0			(uint32_t)0x08000000		// ADC0 Clock Gate Control
#define SIM_SCGC6_FTM1			(uint32_t)0x02000000		// FTM1 Clock Gate Control
#define SIM_SCGC6_FTM0			(uint32_t)0x01000000		// FTM0 Clock Gate Control
#define SIM_SCGC6_PIT			(uint32_t)0x00800000		// PIT Clock Gate Control
#define SIM_SCGC6_PDB			(uint32_t)0x00400000		// PDB Clock Gate Control
#define SIM_SCGC6_USBDCD		(uint32_t)0x00200000		// USB DCD Clock Gate Control
#define SIM_SCGC6_CRC			(uint32_t)0x00040000		// CRC Clock Gate Control
#define SIM_SCGC6_I2S			(uint32_t)0x00008000		// I2S Clock Gate Control
#define SIM_SCGC6_SPI1			(uint32_t)0x00002000		// SPI1 Clock Gate Control
#define SIM_SCGC6_SPI0			(uint32_t)0x00001000		// SPI0 Clock Gate Control
#define SIM_SCGC6_FLEXCAN0		(uint32_t)0x00000010		// FlexCAN0 Clock Gate Control
#define SIM_SCGC6_DMAMUX		(uint32_t)0x00000002		// DMA Mux Clock Gate Control
#define SIM_SCGC6_FTFL			(uint32_t)0x00000001		// Flash Memory Clock Gate Control
#define SIM_SCGC7_DMA			(uint32_t)0x00000002		// DMA Clock Gate Control
#define SIM_CLKDIV2_USBFRAC		(uint32_t)0x01

// Chapter 13: Reset Control Module (RCM)
#define RCM_SRS0		*(volatile uint8_t  *)0x4007F000 // System Reset Status Register 0
#define RCM_SRS1		*(volatile uint8_t  *)0x4007F001 // System Reset Status Register 1
#define RCM_RPFC		*(volatile uint8_t  *)0x4007F004 // Reset Pin Filter Control Register
#define RCM_RPFW		*(volatile uint8_t  *)0x4007F005 // Reset Pin Filter Width Register
#define RCM_MR			*(volatile uint8_t  *)0x4007F007 // Mode Register

// Chapter 14: System Mode Controller
#define SMC_PMPROT		*(volatile uint8_t  *)0x4007E000 // Power Mode Protection Register
#define SMC_PMPROT_AVLP			(uint8_t)0x20			// Allow very low power modes
#define SMC_PMPROT_ALLS			(uint8_t)0x08			// Allow low leakage stop mode
#define SMC_PMPROT_AVLLS		(uint8_t)0x02			// Allow very low leakage stop mode
#define SMC_PMCTRL		*(volatile uint8_t  *)0x4007E001 // Power Mode Control Register
#define SMC_PMCTRL_LPWUI		(uint8_t)0x80			// Low Power Wake Up on Interrupt
#define SMC_PMCTRL_RUNM(n)		(uint8_t)(((n) & 0x03) << 5)	// Run Mode Control
#define SMC_PMCTRL_STOPA		(uint8_t)0x08			// Stop Aborted
#define SMC_PMCTRL_STOPM(n)		(uint8_t)((n) & 0x07)		// Stop Mode Control
#define SMC_VLLSCTRL		*(volatile uint8_t  *)0x4007E002 // VLLS Control Register
#define SMC_VLLSCTRL_PORPO		(uint8_t)0x20			// POR Power Option
#define SMC_VLLSCTRL_VLLSM(n)		(uint8_t)((n) & 0x07)		// VLLS Mode Control
#define SMC_PMSTAT		*(volatile uint8_t  *)0x4007E003 // Power Mode Status Register
#define SMC_PMSTAT_RUN			(uint8_t)0x01			// Current power mode is RUN
#define SMC_PMSTAT_STOP			(uint8_t)0x02			// Current power mode is STOP
#define SMC_PMSTAT_VLPR			(uint8_t)0x04			// Current power mode is VLPR
#define SMC_PMSTAT_VLPW			(uint8_t)0x08			// Current power mode is VLPW
#define SMC_PMSTAT_VLPS			(uint8_t)0x10			// Current power mode is VLPS
#define SMC_PMSTAT_LLS			(uint8_t)0x20			// Current power mode is LLS
#define SMC_PMSTAT_VLLS			(uint8_t)0x40			// Current power mode is VLLS

// Chapter 15: Power Management Controller
#define PMC_LVDSC1_LVDF			(uint8_t)0x80			// Low-Voltage Detect Flag
#define PMC_LVDSC1_LVDACK		(uint8_t)0x40			// Low-Voltage Detect Acknowledge
#define PMC_LVDSC1_LVDIE		(uint8_t)0x20			// Low-Voltage Detect Interrupt Enable
#define PMC_LVDSC1_LVDRE		(uint8_t)0x10			// Low-Voltage Detect Reset Enable
#define PMC_LVDSC2_LVWF			(uint8_t)0x80			// Low-Voltage Warning Flag
#define PMC_LVDSC2_LVWACK		(uint8_t)0x40			// Low-Voltage Warning Acknowledge
#define PMC_LVDSC2_LVWIE		(uint8_t)0x20			// Low-Voltage Warning Interrupt Enable
#define PMC_REGSC_BGEN			(uint8_t)0x10			// Bandgap Enable In VLPx Operation
#define PMC_REGSC_ACKISO		(uint8_t)0x08			// Acknowledge Isolation
#define PMC_REGSC_REGONS		(uint8_t)0x04			// Regulator In Run Regulation Status
#define PMC_REGSC_BGBE			(uint8_t)0x01			// Bandgap Buffer Enable

// Chapter 16: Low-Leakage Wakeup Unit (LLWU)
#define LLWU_FILT1		*(volatile uint8_t  *)0x4007C008 // LLWU Pin Filter 1 register
#define LLWU_FILT2		*(volatile uint8_t  *)0x4007C009 // LLWU Pin Filter 2 register
#define LLWU_RST		*(volatile uint8_t  *)0x4007C00A // LLWU Reset Enable register

// Chapter 17: Miscellaneous Control Module (MCM)
#define MCM_PLACR		*(volatile uint32_t *)0xE008000C // Crossbar Switch (AXBS) Control Register (MK20DX128)
#define MCM_PLACR_ARG			(uint32_t)0x00000200		// Arbitration select, 0=fixed, 1=round-robin
#define MCM_CR			*(volatile uint32_t *)0xE008000C // RAM arbitration control register (MK20DX256)
#define MCM_CR_SRAMLWP			(uint32_t)0x40000000		// SRAM_L write protect
#define MCM_CR_SRAMLAP(n)		(uint32_t)(((n) & 0x03) << 28)	// SRAM_L priority, 0=RR, 1=favor DMA, 2=CPU, 3=DMA
#define MCM_CR_SRAMUWP			(uint32_t)0x04000000		// SRAM_U write protect
#define MCM_CR_SRAMUAP(n)		(uint32_t)(((n) & 0x03) << 24)	// SRAM_U priority, 0=RR, 1=favor DMA, 2=CPU, 3=DMA

// Crossbar Switch (AXBS) - only programmable on MK20DX256
#define AXBS_PRS5		*(volatile uint32_t *)0x40004500 // Priority Registers Slave 5
#define AXBS_CRS5		*(volatile uint32_t *)0x40004510 // Control Register 5
#define AXBS_PRS6		*(volatile uint32_t *)0x40004600 // Priority Registers Slave 6
#define AXBS_CRS6		*(volatile uint32_t *)0x40004610 // Control Register 6
#define AXBS_PRS7		*(volatile uint32_t *)0x40004700 // Priority Registers Slave 7
#define AXBS_CRS7		*(volatile uint32_t *)0x40004710 // Control Register 7
#define AXBS_MGPCR3		*(volatile uint32_t *)0x40004B00 // Master 3 General Purpose Control Register
#define AXBS_MGPCR6		*(volatile uint32_t *)0x40004E00 // Master 6 General Purpose Control Register
#define AXBS_MGPCR7		*(volatile uint32_t *)0x40004F00 // Master 7 General Purpose Control Register
#define AXBS_CRS_READONLY		(uint32_t)0x80000000
#define AXBS_CRS_HALTLOWPRIORITY	(uint32_t)0x40000000
#define AXBS_CRS_ARB_FIXED		(uint32_t)0x00000000
#define AXBS_CRS_ARB_ROUNDROBIN		(uint32_t)0x00010000
#define AXBS_CRS_PARK_FIXED		(uint32_t)0x00000000
#define AXBS_CRS_PARK_PREVIOUS		(uint32_t)0x00000010
#define AXBS_CRS_PARK_NONE		(uint32_t)0x00000020



// Chapter 20: Direct Memory Access Multiplexer (DMAMUX)
#define DMAMUX0_CHCFG0		*(volatile uint8_t  *)0x40021000 // Channel Configuration register
#define DMAMUX0_CHCFG1		*(volatile uint8_t  *)0x40021001 // Channel Configuration register
#define DMAMUX0_CHCFG2		*(volatile uint8_t  *)0x40021002 // Channel Configuration register
#define DMAMUX0_CHCFG3		*(volatile uint8_t  *)0x40021003 // Channel Configuration register
#define DMAMUX0_CHCFG4		*(volatile uint8_t  *)0x40021004 // Channel Configuration register
#define DMAMUX0_CHCFG5		*(volatile uint8_t  *)0x40021005 // Channel Configuration register
#define DMAMUX0_CHCFG6		*(volatile uint8_t  *)0x40021006 // Channel Configuration register
#define DMAMUX0_CHCFG7		*(volatile uint8_t  *)0x40021007 // Channel Configuration register
#define DMAMUX0_CHCFG8		*(volatile uint8_t  *)0x40021008 // Channel Configuration register
#define DMAMUX0_CHCFG9		*(volatile uint8_t  *)0x40021009 // Channel Configuration register
#define DMAMUX0_CHCFG10		*(volatile uint8_t  *)0x4002100A // Channel Configuration register
#define DMAMUX0_CHCFG11		*(volatile uint8_t  *)0x4002100B // Channel Configuration register
#define DMAMUX0_CHCFG12		*(volatile uint8_t  *)0x4002100C // Channel Configuration register
#define DMAMUX0_CHCFG13		*(volatile uint8_t  *)0x4002100D // Channel Configuration register
#define DMAMUX0_CHCFG14		*(volatile uint8_t  *)0x4002100E // Channel Configuration register
#define DMAMUX0_CHCFG15		*(volatile uint8_t  *)0x4002100F // Channel Configuration register
#define DMAMUX_DISABLE			0
#define DMAMUX_TRIG			64
#define DMAMUX_ENABLE			128
#define DMAMUX_SOURCE_UART0_RX		2
#define DMAMUX_SOURCE_UART0_TX		3
#define DMAMUX_SOURCE_UART1_RX		4
#define DMAMUX_SOURCE_UART1_TX		5
#define DMAMUX_SOURCE_UART2_RX		6
#define DMAMUX_SOURCE_UART2_TX		7
#define DMAMUX_SOURCE_I2S0_RX		14
#define DMAMUX_SOURCE_I2S0_TX		15
#define DMAMUX_SOURCE_SPI0_RX		16
#define DMAMUX_SOURCE_SPI0_TX		17
#define DMAMUX_SOURCE_I2C0		22
#define DMAMUX_SOURCE_I2C1		23
#define DMAMUX_SOURCE_FTM0_CH0		24
#define DMAMUX_SOURCE_FTM0_CH1		25
#define DMAMUX_SOURCE_FTM0_CH2		26
#define DMAMUX_SOURCE_FTM0_CH3		27
#define DMAMUX_SOURCE_FTM0_CH4		28
#define DMAMUX_SOURCE_FTM0_CH5		29
#define DMAMUX_SOURCE_FTM0_CH6		30
#define DMAMUX_SOURCE_FTM0_CH7		31
#define DMAMUX_SOURCE_FTM1_CH0		32
#define DMAMUX_SOURCE_FTM1_CH1		33
#define DMAMUX_SOURCE_FTM2_CH0		34
#define DMAMUX_SOURCE_FTM2_CH1		35
#define DMAMUX_SOURCE_ADC0		40
#define DMAMUX_SOURCE_ADC1		41
#define DMAMUX_SOURCE_CMP0		42
#define DMAMUX_SOURCE_CMP1		43
#define DMAMUX_SOURCE_CMP2		44
#define DMAMUX_SOURCE_DAC0		45
#define DMAMUX_SOURCE_CMT		47
#define DMAMUX_SOURCE_PDB		48
#define DMAMUX_SOURCE_PORTA		49
#define DMAMUX_SOURCE_PORTB		50
#define DMAMUX_SOURCE_PORTC		51
#define DMAMUX_SOURCE_PORTD		52
#define DMAMUX_SOURCE_PORTE		53
#define DMAMUX_SOURCE_ALWAYS0		54
#define DMAMUX_SOURCE_ALWAYS1		55
#define DMAMUX_SOURCE_ALWAYS2		56
#define DMAMUX_SOURCE_ALWAYS3		57
#define DMAMUX_SOURCE_ALWAYS4		58
#define DMAMUX_SOURCE_ALWAYS5		59
#define DMAMUX_SOURCE_ALWAYS6		60
#define DMAMUX_SOURCE_ALWAYS7		61
#define DMAMUX_SOURCE_ALWAYS8		62
#define DMAMUX_SOURCE_ALWAYS9		63

// Chapter 21: Direct Memory Access Controller (eDMA)
#define DMA_CR_CX			((uint32_t)(1<<17))	// Cancel Transfer
#define DMA_CR_ECX			((uint32_t)(1<<16))	// Error Cancel Transfer
#define DMA_CR_EMLM			((uint32_t)0x80)	// Enable Minor Loop Mapping
#define DMA_CR_CLM			((uint32_t)0x40)	// Continuous Link Mode
#define DMA_CR_HALT			((uint32_t)0x20)	// Halt DMA Operations
#define DMA_CR_HOE			((uint32_t)0x10)	// Halt On Error
#define DMA_CR_ERCA			((uint32_t)0x04)	// Enable Round Robin Channel Arbitration
#define DMA_CR_EDBG			((uint32_t)0x02)	// Enable Debug
#define DMA_ERQ_ERQ0			((uint32_t)1<<0)	// Enable DMA Request 0
#define DMA_ERQ_ERQ1			((uint32_t)1<<1)	// Enable DMA Request 1
#define DMA_ERQ_ERQ2			((uint32_t)1<<2)	// Enable DMA Request 2
#define DMA_ERQ_ERQ3			((uint32_t)1<<3)	// Enable DMA Request 3
#define DMA_EEI_EEI0			((uint32_t)1<<0)	// Enable Error Interrupt 0
#define DMA_EEI_EEI1			((uint32_t)1<<1)	// Enable Error Interrupt 1
#define DMA_EEI_EEI2			((uint32_t)1<<2)	// Enable Error Interrupt 2
#define DMA_EEI_EEI3			((uint32_t)1<<3)	// Enable Error Interrupt 3
#define DMA_CEEI_CAEE			((uint8_t)1<<6)		// Clear All Enable Error Interrupts
#define DMA_CEEI_NOP			((uint8_t)1<<7)		// NOP
#define DMA_SEEI_SAEE			((uint8_t)1<<6)		// Set All Enable Error Interrupts
#define DMA_SEEI_NOP			((uint8_t)1<<7)		// NOP
#define DMA_CERQ_CAER			((uint8_t)1<<6)		// Clear All Enable Requests
#define DMA_CERQ_NOP			((uint8_t)1<<7)		// NOP
#define DMA_SERQ_SAER			((uint8_t)1<<6)		// Set All Enable Requests
#define DMA_SERQ_NOP			((uint8_t)1<<7)		// NOP
#define DMA_CDNE_CADN			((uint8_t)1<<6)		// Clear All Done Bits
#define DMA_CDNE_NOP			((uint8_t)1<<7)		// NOP
#define DMA_SSRT_SAST			((uint8_t)1<<6)		// Set All Start Bits
#define DMA_SSRT_NOP			((uint8_t)1<<7)		// NOP
#define DMA_CERR_CAEI			((uint8_t)1<<6)		// Clear All Error Indicators
#define DMA_CERR_NOP			((uint8_t)1<<7)		// NOP
#define DMA_CINT_CAIR			((uint8_t)1<<6)		// Clear All Interrupt Requests
#define DMA_CINT_NOP			((uint8_t)1<<7)		// NOP
#define DMA_INT_INT0			((uint32_t)1<<0)	// Interrupt Request 0
#define DMA_INT_INT1			((uint32_t)1<<1)	// Interrupt Request 1
#define DMA_INT_INT2			((uint32_t)1<<2)	// Interrupt Request 2
#define DMA_INT_INT3			((uint32_t)1<<3)	// Interrupt Request 3
#define DMA_ERR_ERR0			((uint32_t)1<<0)	// Error in Channel 0
#define DMA_ERR_ERR1			((uint32_t)1<<1)	// Error in Channel 1
#define DMA_ERR_ERR2			((uint32_t)1<<2)	// Error in Channel 2
#define DMA_ERR_ERR3			((uint32_t)1<<3)	// Error in Channel 3
#define DMA_HRS_HRS0			((uint32_t)1<<0)	// Hardware Request Status Channel 0
#define DMA_HRS_HRS1			((uint32_t)1<<1)	// Hardware Request Status Channel 1
#define DMA_HRS_HRS2			((uint32_t)1<<2)	// Hardware Request Status Channel 2
#define DMA_HRS_HRS3			((uint32_t)1<<3)	// Hardware Request Status Channel 3
#define DMA_DCHPRI_CHPRI(n)		((uint8_t)(n & 3)<<0)	// Channel Arbitration Priority
#define DMA_DCHPRI_DPA			((uint8_t)1<<6)		// Disable PreEmpt Ability
#define DMA_DCHPRI_ECP			((uint8_t)1<<7)		// Enable PreEmption


#define DMA_TCD_ATTR_SMOD(n)		(((n) & 0x1F) << 11)
#define DMA_TCD_ATTR_SSIZE(n)		(((n) & 0x7) << 8)
#define DMA_TCD_ATTR_DMOD(n)		(((n) & 0x1F) << 3)
#define DMA_TCD_ATTR_DSIZE(n)		(((n) & 0x7) << 0)
#define DMA_TCD_ATTR_SIZE_8BIT		0
#define DMA_TCD_ATTR_SIZE_16BIT		1
#define DMA_TCD_ATTR_SIZE_32BIT		2
#define DMA_TCD_ATTR_SIZE_16BYTE	4
#define DMA_TCD_ATTR_SIZE_32BYTE	5
#define DMA_TCD_CSR_BWC(n)		(((n) & 0x3) << 14)
#define DMA_TCD_CSR_MAJORLINKCH(n)	(((n) & 0x3) << 8)
#define DMA_TCD_CSR_DONE		0x0080
#define DMA_TCD_CSR_ACTIVE		0x0040
#define DMA_TCD_CSR_MAJORELINK		0x0020
#define DMA_TCD_CSR_ESG			0x0010
#define DMA_TCD_CSR_DREQ		0x0008
#define DMA_TCD_CSR_INTHALF		0x0004
#define DMA_TCD_CSR_INTMAJOR		0x0002
#define DMA_TCD_CSR_START		0x0001
#define DMA_TCD_CITER_MASK		    ((uint16_t)0x7FFF)	   // Loop count mask
#define DMA_TCD_CITER_ELINK		    ((uint16_t)1<<15)	   // Enable channel linking on minor-loop complete
#define DMA_TCD_BITER_MASK		    ((uint16_t)0x7FFF)	   // Loop count mask
#define DMA_TCD_BITER_ELINK		    ((uint16_t)1<<15)	   // Enable channel linking on minor-loop complete
#define DMA_TCD_NBYTES_SMLOE		    ((uint32_t)1<<31)		    // Source Minor Loop Offset Enable
#define DMA_TCD_NBYTES_DMLOE		    ((uint32_t)1<<30)		    // Destination Minor Loop Offset Enable
#define DMA_TCD_NBYTES_MLOFFNO_NBYTES(n)    ((uint32_t)(n))		    // NBytes transfer count when minor loop disabled
#define DMA_TCD_NBYTES_MLOFFYES_NBYTES(n)   ((uint32_t)(n & 0x1F))	    // NBytes transfer count when minor loop enabled
#define DMA_TCD_NBYTES_MLOFFYES_MLOFF(n)    ((uint32_t)(n & 0xFFFFF)<<10)   // Offset 


















// Chapter 22: External Watchdog Monitor (EWM)

// Chapter 23: Watchdog Timer (WDOG)
#define WDOG_STCTRLH_DISTESTWDOG	(uint16_t)0x4000		// Allows the WDOG's functional test mode to be disabled permanently.
#define WDOG_STCTRLH_TESTSEL		(uint16_t)0x0800
#define WDOG_STCTRLH_TESTWDOG		(uint16_t)0x0400
#define WDOG_STCTRLH_WAITEN		(uint16_t)0x0080
#define WDOG_STCTRLH_STOPEN		(uint16_t)0x0040
#define WDOG_STCTRLH_DBGEN		(uint16_t)0x0020
#define WDOG_STCTRLH_ALLOWUPDATE	(uint16_t)0x0010
#define WDOG_STCTRLH_WINEN		(uint16_t)0x0008
#define WDOG_STCTRLH_IRQRSTEN		(uint16_t)0x0004
#define WDOG_STCTRLH_CLKSRC		(uint16_t)0x0002
#define WDOG_STCTRLH_WDOGEN		(uint16_t)0x0001
#define WDOG_UNLOCK_SEQ1		(uint16_t)0xC520
#define WDOG_UNLOCK_SEQ2		(uint16_t)0xD928

// Chapter 24: Multipurpose Clock Generator (MCG)
#define MCG_C1_IREFSTEN			(uint8_t)0x01			// Internal Reference Stop Enable, Controls whether or not the internal reference clock remains enabled when the MCG enters Stop mode.
#define MCG_C1_IRCLKEN			(uint8_t)0x02			// Internal Reference Clock Enable, Enables the internal reference clock for use as MCGIRCLK.
#define MCG_C1_IREFS			(uint8_t)0x04			// Internal Reference Select, Selects the reference clock source for the FLL.
#define MCG_C2_IRCS			(uint8_t)0x01			// Internal Reference Clock Select, Selects between the fast or slow internal reference clock source.
#define MCG_C2_LP			(uint8_t)0x02			// Low Power Select, Controls whether the FLL or PLL is disabled in BLPI and BLPE modes.
#define MCG_C2_EREFS			(uint8_t)0x04			// External Reference Select, Selects the source for the external reference clock. 
#define MCG_C2_HGO0			(uint8_t)0x08			// High Gain Oscillator Select, Controls the crystal oscillator mode of operation
#define MCG_C2_RANGE0(n)		(uint8_t)(((n) & 0x03) << 4)	// Frequency Range Select, Selects the frequency range for the crystal oscillator
#define MCG_C2_LOCRE0			(uint8_t)0x80			// Loss of Clock Reset Enable, Determines whether an interrupt or a reset request is made following a loss of OSC0 
#define MCG_C4_SCFTRIM			(uint8_t)0x01			// Slow Internal Reference Clock Fine Trim
#define MCG_C4_DMX32			(uint8_t)0x80			// DCO Maximum Frequency with 32.768 kHz Reference, controls whether the DCO frequency range is narrowed
#define MCG_C5_PRDIV0(n)		(uint8_t)((n) & 0x1F)		// PLL External Reference Divider
#define MCG_C5_PLLSTEN0			(uint8_t)0x20			// PLL Stop Enable
#define MCG_C5_PLLCLKEN0		(uint8_t)0x40			// PLL Clock Enable
#define MCG_C6_VDIV0(n)			(uint8_t)((n) & 0x1F)		// VCO 0 Divider
#define MCG_C6_CME0			(uint8_t)0x20			// Clock Monitor Enable
#define MCG_C6_PLLS			(uint8_t)0x40			// PLL Select, Controls whether the PLL or FLL output is selected as the MCG source when CLKS[1:0]=00. 
#define MCG_C6_LOLIE0			(uint8_t)0x80			// Loss of Lock Interrrupt Enable
#define MCG_S_IRCST			(uint8_t)0x01			// Internal Reference Clock Status
#define MCG_S_OSCINIT0			(uint8_t)0x02			// OSC Initialization,	resets to 0, is set to 1 after the initialization cycles of the crystal oscillator
#define MCG_S_IREFST			(uint8_t)0x10			// Internal Reference Status
#define MCG_S_PLLST			(uint8_t)0x20			// PLL Select Status
#define MCG_S_LOCK0			(uint8_t)0x40			// Lock Status, 0=PLL Unlocked, 1=PLL Locked
#define MCG_S_LOLS0			(uint8_t)0x80			// Loss of Lock Status
#define MCG_SC			*(volatile uint8_t  *)0x40064008 // MCG Status and Control Register
#define MCG_SC_LOCS0			(uint8_t)0x01			// OSC0 Loss of Clock Status
#define MCG_SC_FCRDIV(n)		(uint8_t)(((n) & 0x07) << 1)	// Fast Clock Internal Reference Divider
#define MCG_SC_FLTPRSRV			(uint8_t)0x10			// FLL Filter Preserve Enable
#define MCG_SC_ATMF			(uint8_t)0x20			// Automatic Trim Machine Fail Flag
#define MCG_SC_ATMS			(uint8_t)0x40			// Automatic Trim Machine Select
#define MCG_SC_ATME			(uint8_t)0x80			// Automatic Trim Machine Enable
#define MCG_C7			*(volatile uint8_t  *)0x4006400C // MCG Control 7 Register
#define MCG_C8			*(volatile uint8_t  *)0x4006400D // MCG Control 8 Register

// Chapter 25: Oscillator (OSC)
#define OSC0_CR			*(volatile uint8_t  *)0x40065000 // OSC Control Register
#define OSC_SC16P			(uint8_t)0x01			// Oscillator 16 pF Capacitor Load Configure
#define OSC_SC8P			(uint8_t)0x02			// Oscillator 8 pF Capacitor Load Configure
#define OSC_SC4P			(uint8_t)0x04			// Oscillator 4 pF Capacitor Load Configure
#define OSC_SC2P			(uint8_t)0x08			// Oscillator 2 pF Capacitor Load Configure
#define OSC_EREFSTEN			(uint8_t)0x20			// External Reference Stop Enable, Controls whether or not the external reference clock (OSCERCLK) remains enabled when MCU enters Stop mode.
#define OSC_ERCLKEN			(uint8_t)0x80			// External Reference Enable, Enables external reference clock (OSCERCLK).

// Chapter 27: Flash Memory Controller (FMC)
#define FMC_DATAW0S0		*(volatile uint32_t *)0x4001F200	// Cache Data Storage
#define FMC_DATAW0S1		*(volatile uint32_t *)0x4001F204	// Cache Data Storage
#define FMC_DATAW1S0		*(volatile uint32_t *)0x4001F208	// Cache Data Storage
#define FMC_DATAW1S1		*(volatile uint32_t *)0x4001F20C	// Cache Data Storage
#define FMC_DATAW2S0		*(volatile uint32_t *)0x4001F210	// Cache Data Storage
#define FMC_DATAW2S1		*(volatile uint32_t *)0x4001F214	// Cache Data Storage
#define FMC_DATAW3S0		*(volatile uint32_t *)0x4001F218	// Cache Data Storage
#define FMC_DATAW3S1		*(volatile uint32_t *)0x4001F21C	// Cache Data Storage

// Chapter 28: Flash Memory Module (FTFL)
#define FTFL_FSTAT_CCIF			(uint8_t)0x80			// Command Complete Interrupt Flag
#define FTFL_FSTAT_RDCOLERR		(uint8_t)0x40			// Flash Read Collision Error Flag
#define FTFL_FSTAT_ACCERR		(uint8_t)0x20			// Flash Access Error Flag
#define FTFL_FSTAT_FPVIOL		(uint8_t)0x10			// Flash Protection Violation Flag
#define FTFL_FSTAT_MGSTAT0		(uint8_t)0x01			// Memory Controller Command Completion Status Flag
#define FTFL_FCNFG_CCIE			(uint8_t)0x80			// Command Complete Interrupt Enable
#define FTFL_FCNFG_RDCOLLIE		(uint8_t)0x40			// Read Collision Error Interrupt Enable
#define FTFL_FCNFG_ERSAREQ		(uint8_t)0x20			// Erase All Request
#define FTFL_FCNFG_ERSSUSP		(uint8_t)0x10			// Erase Suspend
#define FTFL_FCNFG_PFLSH		(uint8_t)0x04			// Flash memory configuration
#define FTFL_FCNFG_RAMRDY		(uint8_t)0x02			// RAM Ready
#define FTFL_FCNFG_EEERDY		(uint8_t)0x01			// EEPROM Ready

// Chapter 30: Cyclic Redundancy Check (CRC)

// Chapter 31: Analog-to-Digital Converter (ADC)
#define ADC_SC1_COCO			(uint32_t)0x80			// Conversion complete flag
#define ADC_SC1_AIEN			(uint32_t)0x40			// Interrupt enable
#define ADC_SC1_DIFF			(uint32_t)0x20			// Differential mode enable
#define ADC_CFG1_ADLPC			(uint32_t)0x80			// Low-power configuration
#define ADC_CFG1_ADLSMP			(uint32_t)0x10			// Sample time configuration, 0=Short, 1=Long
#define ADC_CFG2_MUXSEL			(uint32_t)0x10			// 0=a channels, 1=b channels
#define ADC_CFG2_ADACKEN		(uint32_t)0x08			// async clock enable
#define ADC_CFG2_ADHSC			(uint32_t)0x04			// High speed configuration
#define ADC_SC2_ADACT			(uint32_t)0x80			// Conversion active
#define ADC_SC2_ADTRG			(uint32_t)0x40			// Conversion trigger select, 0=software, 1=hardware
#define ADC_SC2_ACFE			(uint32_t)0x20			// Compare function enable
#define ADC_SC2_ACFGT			(uint32_t)0x10			// Compare function greater than enable
#define ADC_SC2_ACREN			(uint32_t)0x08			// Compare function range enable
#define ADC_SC2_DMAEN			(uint32_t)0x04			// DMA enable
#define ADC_SC3_CAL			(uint32_t)0x80			// Calibration, 1=begin, stays set while cal in progress
#define ADC_SC3_CALF			(uint32_t)0x40			// Calibration failed flag
#define ADC_SC3_ADCO			(uint32_t)0x08			// Continuous conversion enable
#define ADC_SC3_AVGE			(uint32_t)0x04			// Hardware average enable
#define ADC0_PGA_PGAEN			(uint32_t)0x00800000		// Enable
#define ADC0_PGA_PGALPB			(uint32_t)0x00100000		// Low-Power Mode Control, 0=low power, 1=normal
#define ADC0_PGA_PGAG(n)		(uint32_t)(((n) & 15) << 16)	// Gain, 0=1X, 1=2X, 2=4X, 3=8X, 4=16X, 5=32X, 6=64X


#define DAC_C0_DACEN			0x80				// DAC Enable
#define DAC_C0_DACRFS			0x40				// DAC Reference Select
#define DAC_C0_DACTRGSEL		0x20				// DAC Trigger Select
#define DAC_C0_DACSWTRG			0x10				// DAC Software Trigger
#define DAC_C0_LPEN			0x08				// DAC Low Power Control
#define DAC_C0_DACBWIEN			0x04				// DAC Buffer Watermark Interrupt Enable
#define DAC_C0_DACBTIEN			0x02				// DAC Buffer Read Pointer Top Flag Interrupt Enable
#define DAC_C0_DACBBIEN			0x01				// DAC Buffer Read Pointer Bottom Flag Interrupt Enable
#define DAC_C1_DMAEN			0x80				// DMA Enable Select
#define DAC_C1_DACBFEN			0x00				// DAC Buffer Enable



//#define MCG_C2_RANGE0(n)		(uint8_t)(((n) & 0x03) << 4)	// Frequency Range Select, Selects the frequency range for the crystal oscillator
//#define MCG_C2_LOCRE0			(uint8_t)0x80			// Loss of Clock Reset Enable, Determines whether an interrupt or a reset request is made following a loss of OSC0 

// Chapter 32: Comparator (CMP)

// Chapter 33: Voltage Reference (VREFV1)

// Chapter 34: Programmable Delay Block (PDB)
#define PDB_SC_PDBEIE			0x00020000		// Sequence Error Interrupt Enable
#define PDB_SC_SWTRIG			0x00010000		// Software Trigger
#define PDB_SC_DMAEN			0x00008000		// DMA Enable
#define PDB_SC_PDBEN			0x00000080		// PDB Enable
#define PDB_SC_PDBIF			0x00000040		// PDB Interrupt Flag
#define PDB_SC_PDBIE			0x00000020		// PDB Interrupt Enable.
#define PDB_SC_CONT			0x00000002		// Continuous Mode Enable
#define PDB_SC_LDOK			0x00000001		// Load OK
#define PDB0_POEN		*(volatile uint32_t *)0x40036190 // Pulse-Out n Enable Register
#define PDB0_PO1DLY		*(volatile uint32_t *)0x40036198 // Pulse-Out n Delay Register

// Chapter 35: FlexTimer Module (FTM)
#define FTM_SC_TOF			0x80				// Timer Overflow Flag
#define FTM_SC_TOIE			0x40				// Timer Overflow Interrupt Enable
#define FTM_SC_CPWMS			0x20				// Center-Aligned PWM Select
#define FTM_MODE_FAULTIE		0x80				// Fault Interrupt Enable
#define FTM_MODE_CAPTEST		0x10				// Capture Test Mode Enable
#define FTM_MODE_PWMSYNC		0x08				// PWM Synchronization Mode
#define FTM_MODE_WPDIS			0x04				// Write Protection Disable
#define FTM_MODE_INIT			0x02				// Initialize The Channels Output
#define FTM_MODE_FTMEN			0x01				// FTM Enable
#define FTM_SYNC_SWSYNC			0x80				// 
#define FTM_SYNC_TRIG2			0x40				// 
#define FTM_SYNC_TRIG1			0x20				// 
#define FTM_SYNC_TRIG0			0x10				// 
#define FTM_SYNC_SYNCHOM		0x08				// 
#define FTM_SYNC_REINIT			0x04				// 
#define FTM_SYNC_CNTMAX			0x02				// 
#define FTM_SYNC_CNTMIN			0x01				// 

// Chapter 36: Periodic Interrupt Timer (PIT)

// Chapter 37: Low-Power Timer (LPTMR)

// Chapter 38: Carrier Modulator Transmitter (CMT)

// Chapter 39: Real Time Clock (RTC)
#define RTC_CR_SC2P			(uint32_t)0x00002000		// 
#define RTC_CR_SC4P			(uint32_t)0x00001000		// 
#define RTC_CR_SC8P			(uint32_t)0x00000800		// 
#define RTC_CR_SC16P			(uint32_t)0x00000400		// 
#define RTC_CR_CLKO			(uint32_t)0x00000200		// 
#define RTC_CR_OSCE			(uint32_t)0x00000100		// 
#define RTC_CR_UM			(uint32_t)0x00000008		// 
#define RTC_CR_SUP			(uint32_t)0x00000004		// 
#define RTC_CR_WPE			(uint32_t)0x00000002		// 
#define RTC_CR_SWR			(uint32_t)0x00000001		// 
#define RTC_SR_TCE			(uint32_t)0x00000010		// 
#define RTC_SR_TAF			(uint32_t)0x00000004		// 
#define RTC_SR_TOF			(uint32_t)0x00000002		// 
#define RTC_SR_TIF			(uint32_t)0x00000001		// 

// Chapter 40: Universal Serial Bus OTG Controller (USBOTG)
#define USB_OTGISTAT_IDCHG		(uint8_t)0x80			//
#define USB_OTGISTAT_ONEMSEC		(uint8_t)0x40			//
#define USB_OTGISTAT_LINE_STATE_CHG	(uint8_t)0x20			//
#define USB_OTGISTAT_SESSVLDCHG		(uint8_t)0x08			//
#define USB_OTGISTAT_B_SESS_CHG		(uint8_t)0x04			//
#define USB_OTGISTAT_AVBUSCHG		(uint8_t)0x01			//
#define USB_OTGICR_IDEN			(uint8_t)0x80			// 
#define USB_OTGICR_ONEMSECEN		(uint8_t)0x40			// 
#define USB_OTGICR_LINESTATEEN		(uint8_t)0x20			// 
#define USB_OTGICR_SESSVLDEN		(uint8_t)0x08			// 
#define USB_OTGICR_BSESSEN		(uint8_t)0x04			// 
#define USB_OTGICR_AVBUSEN		(uint8_t)0x01			// 
#define USB_OTGSTAT_ID			(uint8_t)0x80			// 
#define USB_OTGSTAT_ONEMSECEN		(uint8_t)0x40			// 
#define USB_OTGSTAT_LINESTATESTABLE	(uint8_t)0x20			// 
#define USB_OTGSTAT_SESS_VLD		(uint8_t)0x08			// 
#define USB_OTGSTAT_BSESSEND		(uint8_t)0x04			// 
#define USB_OTGSTAT_AVBUSVLD		(uint8_t)0x01			// 
#define USB_OTGCTL_DPHIGH		(uint8_t)0x80			// 
#define USB_OTGCTL_DPLOW		(uint8_t)0x20			// 
#define USB_OTGCTL_DMLOW		(uint8_t)0x10			// 
#define USB_OTGCTL_OTGEN		(uint8_t)0x04			// 
#define USB_ISTAT_STALL			(uint8_t)0x80			// 
#define USB_ISTAT_ATTACH		(uint8_t)0x40			// 
#define USB_ISTAT_RESUME		(uint8_t)0x20			// 
#define USB_ISTAT_SLEEP			(uint8_t)0x10			// 
#define USB_ISTAT_TOKDNE		(uint8_t)0x08			// 
#define USB_ISTAT_SOFTOK		(uint8_t)0x04			// 
#define USB_ISTAT_ERROR			(uint8_t)0x02			// 
#define USB_ISTAT_USBRST		(uint8_t)0x01			// 
#define USB_INTEN_STALLEN		(uint8_t)0x80			// 
#define USB_INTEN_ATTACHEN		(uint8_t)0x40			// 
#define USB_INTEN_RESUMEEN		(uint8_t)0x20			// 
#define USB_INTEN_SLEEPEN		(uint8_t)0x10			// 
#define USB_INTEN_TOKDNEEN		(uint8_t)0x08			// 
#define USB_INTEN_SOFTOKEN		(uint8_t)0x04			// 
#define USB_INTEN_ERROREN		(uint8_t)0x02			// 
#define USB_INTEN_USBRSTEN		(uint8_t)0x01			// 
#define USB_ERRSTAT_BTSERR		(uint8_t)0x80			// 
#define USB_ERRSTAT_DMAERR		(uint8_t)0x20			// 
#define USB_ERRSTAT_BTOERR		(uint8_t)0x10			// 
#define USB_ERRSTAT_DFN8		(uint8_t)0x08			// 
#define USB_ERRSTAT_CRC16		(uint8_t)0x04			// 
#define USB_ERRSTAT_CRC5EOF		(uint8_t)0x02			// 
#define USB_ERRSTAT_PIDERR		(uint8_t)0x01			// 
#define USB_ERREN_BTSERREN		(uint8_t)0x80			// 
#define USB_ERREN_DMAERREN		(uint8_t)0x20			// 
#define USB_ERREN_BTOERREN		(uint8_t)0x10			// 
#define USB_ERREN_DFN8EN		(uint8_t)0x08			// 
#define USB_ERREN_CRC16EN		(uint8_t)0x04			// 
#define USB_ERREN_CRC5EOFEN		(uint8_t)0x02			// 
#define USB_ERREN_PIDERREN		(uint8_t)0x01			// 
#define USB_STAT_TX			(uint8_t)0x08			// 
#define USB_STAT_ODD			(uint8_t)0x04			// 
#define USB_CTL_JSTATE			(uint8_t)0x80			// 
#define USB_CTL_SE0			(uint8_t)0x40			// 
#define USB_CTL_TXSUSPENDTOKENBUSY	(uint8_t)0x20			// 
#define USB_CTL_RESET			(uint8_t)0x10			// 
#define USB_CTL_HOSTMODEEN		(uint8_t)0x08			// 
#define USB_CTL_RESUME			(uint8_t)0x04			// 
#define USB_CTL_ODDRST			(uint8_t)0x02			// 
#define USB_CTL_USBENSOFEN		(uint8_t)0x01			// 
#define USB_ENDPT_HOSTWOHUB		(uint8_t)0x80			// host only, enable low speed
#define USB_ENDPT_RETRYDIS		(uint8_t)0x40			// host only, set to disable NAK retry
#define USB_ENDPT_EPCTLDIS		(uint8_t)0x10			// 0=control, 1=bulk, interrupt, isync
#define USB_ENDPT_EPRXEN		(uint8_t)0x08			// enables the endpoint for RX transfers.
#define USB_ENDPT_EPTXEN		(uint8_t)0x04			// enables the endpoint for TX transfers.
#define USB_ENDPT_EPSTALL		(uint8_t)0x02			// set to stall endpoint
#define USB_ENDPT_EPHSHK		(uint8_t)0x01			// enable handshaking during a transaction, generally set unless Isochronous
#define USB_USBCTRL_SUSP		(uint8_t)0x80			// Places the USB transceiver into the suspend state.
#define USB_USBCTRL_PDE			(uint8_t)0x40			// Enables the weak pulldowns on the USB transceiver.
#define USB_OBSERVE_DPPU		(uint8_t)0x80			// 
#define USB_OBSERVE_DPPD		(uint8_t)0x40			// 
#define USB_OBSERVE_DMPD		(uint8_t)0x10			// 
#define USB_CONTROL_DPPULLUPNONOTG	(uint8_t)0x10			//  Provides control of the DP PULLUP in the USB OTG module, if USB is configured in non-OTG device mode.
#define USB_USBTRC_USBRESET		(uint8_t)0x80			//
#define USB_USBTRC_USBRESMEN		(uint8_t)0x20			//
#define USB_USBTRC_SYNC_DET		(uint8_t)0x02			//
#define USB_USBTRC_USB_RESUME_INT	(uint8_t)0x01			//
#define USB0_USBFRMADJUST	*(volatile uint8_t  *)0x40072114 // Frame Adjust Register

// Chapter 41: USB Device Charger Detection Module (USBDCD)

// Chapter 43: SPI (DSPI)
#define SPI_MCR_MSTR			(uint32_t)0x80000000		// Master/Slave Mode Select
#define SPI_MCR_CONT_SCKE		(uint32_t)0x40000000		// 
#define SPI_MCR_FRZ			(uint32_t)0x08000000		// 
#define SPI_MCR_MTFE			(uint32_t)0x04000000		// 
#define SPI_MCR_ROOE			(uint32_t)0x01000000		// 
#define SPI_MCR_DOZE			(uint32_t)0x00008000		// 
#define SPI_MCR_MDIS			(uint32_t)0x00004000		// 
#define SPI_MCR_DIS_TXF			(uint32_t)0x00002000		// 
#define SPI_MCR_DIS_RXF			(uint32_t)0x00001000		// 
#define SPI_MCR_CLR_TXF			(uint32_t)0x00000800		// 
#define SPI_MCR_CLR_RXF			(uint32_t)0x00000400		// 
#define SPI_MCR_HALT			(uint32_t)0x00000001		// 
#define SPI_CTAR_DBR			(uint32_t)0x80000000		// Double Baud Rate
#define SPI_CTAR_CPOL			(uint32_t)0x04000000		// Clock Polarity
#define SPI_CTAR_CPHA			(uint32_t)0x02000000		// Clock Phase
#define SPI_CTAR_LSBFE			(uint32_t)0x01000000		// LSB First
#define SPI_SR_TCF			(uint32_t)0x80000000		// Transfer Complete Flag
#define SPI_SR_TXRXS			(uint32_t)0x40000000		// TX and RX Status
#define SPI_SR_EOQF			(uint32_t)0x10000000		// End of Queue Flag
#define SPI_SR_TFUF			(uint32_t)0x08000000		// Transmit FIFO Underflow Flag
#define SPI_SR_TFFF			(uint32_t)0x02000000		// Transmit FIFO Fill Flag
#define SPI_SR_RFOF			(uint32_t)0x00080000		// Receive FIFO Overflow Flag
#define SPI_SR_RFDF			(uint32_t)0x00020000		// Receive FIFO Drain Flag
#define SPI_RSER_TCF_RE			(uint32_t)0x80000000		// Transmission Complete Request Enable
#define SPI_RSER_EOQF_RE		(uint32_t)0x10000000		// DSPI Finished Request Request Enable
#define SPI_RSER_TFUF_RE		(uint32_t)0x08000000		// Transmit FIFO Underflow Request Enable
#define SPI_RSER_TFFF_RE		(uint32_t)0x02000000		// Transmit FIFO Fill Request Enable
#define SPI_RSER_TFFF_DIRS		(uint32_t)0x01000000		// Transmit FIFO FIll Dma or Interrupt Request Select
#define SPI_RSER_RFOF_RE		(uint32_t)0x00080000		// Receive FIFO Overflow Request Enable
#define SPI_RSER_RFDF_RE		(uint32_t)0x00020000		// Receive FIFO Drain Request Enable
#define SPI_RSER_RFDF_DIRS		(uint32_t)0x00010000		// Receive FIFO Drain DMA or Interrupt Request Select
#define SPI_PUSHR_CONT			(uint32_t)0x80000000		// 
#define SPI_PUSHR_EOQ			(uint32_t)0x08000000		// 
#define SPI_PUSHR_CTCNT			(uint32_t)0x04000000		// 
typedef struct {
	volatile uint32_t	MCR;	// 0
	volatile uint32_t	unused1;// 4
	volatile uint32_t	TCR;	// 8
	volatile uint32_t	CTAR0;	// c
	volatile uint32_t	CTAR1;	// 10
	volatile uint32_t	CTAR2;	// 14
	volatile uint32_t	CTAR3;	// 18
	volatile uint32_t	CTAR4;	// 1c
	volatile uint32_t	CTAR5;	// 20
	volatile uint32_t	CTAR6;	// 24
	volatile uint32_t	CTAR7;	// 28
	volatile uint32_t	SR;	// 2c
	volatile uint32_t	RSER;	// 30
	volatile uint32_t	PUSHR;	// 34
	volatile uint32_t	POPR;	// 38
	volatile uint32_t	TXFR[16]; // 3c
	volatile uint32_t	RXFR[16]; // 7c
} SPI_t;
#define SPI0		(*(SPI_t *)0x4002C000)

// Chapter 44: Inter-Integrated Circuit (I2C)
#define I2C_C1_IICEN			(uint8_t)0x80			// I2C Enable
#define I2C_C1_IICIE			(uint8_t)0x40			// I2C Interrupt Enable
#define I2C_C1_MST			(uint8_t)0x20			// Master Mode Select
#define I2C_C1_TX			(uint8_t)0x10			// Transmit Mode Select
#define I2C_C1_TXAK			(uint8_t)0x08			// Transmit Acknowledge Enable
#define I2C_C1_RSTA			(uint8_t)0x04			// Repeat START
#define I2C_C1_WUEN			(uint8_t)0x02			// Wakeup Enable
#define I2C_C1_DMAEN			(uint8_t)0x01			// DMA Enable
#define I2C_S_TCF			(uint8_t)0x80			// Transfer Complete Flag
#define I2C_S_IAAS			(uint8_t)0x40			// Addressed As A Slave
#define I2C_S_BUSY			(uint8_t)0x20			// Bus Busy
#define I2C_S_ARBL			(uint8_t)0x10			// Arbitration Lost
#define I2C_S_RAM			(uint8_t)0x08			// Range Address Match
#define I2C_S_SRW			(uint8_t)0x04			// Slave Read/Write
#define I2C_S_IICIF			(uint8_t)0x02			// Interrupt Flag
#define I2C_S_RXAK			(uint8_t)0x01			// Receive Acknowledge
#define I2C_C2_GCAEN			(uint8_t)0x80			// General Call Address Enable
#define I2C_C2_ADEXT			(uint8_t)0x40			// Address Extension
#define I2C_C2_HDRS			(uint8_t)0x20			// High Drive Select
#define I2C_C2_SBRC			(uint8_t)0x10			// Slave Baud Rate Control
#define I2C_C2_RMEN			(uint8_t)0x08			// Range Address Matching Enable


// Chapter 45: Universal Asynchronous Receiver/Transmitter (UART)
#define UART_C1_LOOPS			(uint8_t)0x80			// When LOOPS is set, the RxD pin is disconnected from the UART and the transmitter output is internally connected to the receiver input
#define UART_C1_UARTSWAI		(uint8_t)0x40			// UART Stops in Wait Mode
#define UART_C1_RSRC			(uint8_t)0x20			// When LOOPS is set, the RSRC field determines the source for the receiver shift register input
#define UART_C1_M			(uint8_t)0x10			// 9-bit or 8-bit Mode Select
#define UART_C1_WAKE			(uint8_t)0x08			// Determines which condition wakes the UART
#define UART_C1_ILT			(uint8_t)0x04			// Idle Line Type Select
#define UART_C1_PE			(uint8_t)0x02			// Parity Enable
#define UART_C1_PT			(uint8_t)0x01			// Parity Type, 0=even, 1=odd
#define UART_C2_TIE			(uint8_t)0x80			// Transmitter Interrupt or DMA Transfer Enable.
#define UART_C2_TCIE			(uint8_t)0x40			// Transmission Complete Interrupt Enable
#define UART_C2_RIE			(uint8_t)0x20			// Receiver Full Interrupt or DMA Transfer Enable
#define UART_C2_ILIE			(uint8_t)0x10			// Idle Line Interrupt Enable
#define UART_C2_TE			(uint8_t)0x08			// Transmitter Enable
#define UART_C2_RE			(uint8_t)0x04			// Receiver Enable
#define UART_C2_RWU			(uint8_t)0x02			// Receiver Wakeup Control
#define UART_C2_SBK			(uint8_t)0x01			// Send Break
#define UART_S1_TDRE			(uint8_t)0x80			// Transmit Data Register Empty Flag
#define UART_S1_TC			(uint8_t)0x40			// Transmit Complete Flag
#define UART_S1_RDRF			(uint8_t)0x20			// Receive Data Register Full Flag
#define UART_S1_IDLE			(uint8_t)0x10			// Idle Line Flag
#define UART_S1_OR			(uint8_t)0x08			// Receiver Overrun Flag
#define UART_S1_NF			(uint8_t)0x04			// Noise Flag
#define UART_S1_FE			(uint8_t)0x02			// Framing Error Flag
#define UART_S1_PF			(uint8_t)0x01			// Parity Error Flag
#define UART_PFIFO_TXFE			(uint8_t)0x80
#define UART_PFIFO_RXFE			(uint8_t)0x08
#define UART_CFIFO_TXFLUSH		(uint8_t)0x80			// 
#define UART_CFIFO_RXFLUSH		(uint8_t)0x40			// 
#define UART_CFIFO_RXOFE		(uint8_t)0x04			// 
#define UART_CFIFO_TXOFE		(uint8_t)0x02			// 
#define UART_CFIFO_RXUFE		(uint8_t)0x01			// 
#define UART_SFIFO_TXEMPT		(uint8_t)0x80
#define UART_SFIFO_RXEMPT		(uint8_t)0x40
#define UART_SFIFO_RXOF			(uint8_t)0x04
#define UART_SFIFO_TXOF			(uint8_t)0x02
#define UART_SFIFO_RXUF			(uint8_t)0x01
#define UART0_C6		*(volatile uint8_t  *)0x4006A021 // UART CEA709.1-B Control Register 6
#define UART0_PCTH		*(volatile uint8_t  *)0x4006A022 // UART CEA709.1-B Packet Cycle Time Counter High
#define UART0_PCTL		*(volatile uint8_t  *)0x4006A023 // UART CEA709.1-B Packet Cycle Time Counter Low
#define UART0_B1T		*(volatile uint8_t  *)0x4006A024 // UART CEA709.1-B Beta1 Timer
#define UART0_SDTH		*(volatile uint8_t  *)0x4006A025 // UART CEA709.1-B Secondary Delay Timer High
#define UART0_SDTL		*(volatile uint8_t  *)0x4006A026 // UART CEA709.1-B Secondary Delay Timer Low
#define UART0_PRE		*(volatile uint8_t  *)0x4006A027 // UART CEA709.1-B Preamble
#define UART0_TPL		*(volatile uint8_t  *)0x4006A028 // UART CEA709.1-B Transmit Packet Length
#define UART0_IE		*(volatile uint8_t  *)0x4006A029 // UART CEA709.1-B Interrupt Enable Register
#define UART0_WB		*(volatile uint8_t  *)0x4006A02A // UART CEA709.1-B WBASE
#define UART0_S3		*(volatile uint8_t  *)0x4006A02B // UART CEA709.1-B Status Register
#define UART0_S4		*(volatile uint8_t  *)0x4006A02C // UART CEA709.1-B Status Register
#define UART0_RPL		*(volatile uint8_t  *)0x4006A02D // UART CEA709.1-B Received Packet Length
#define UART0_RPREL		*(volatile uint8_t  *)0x4006A02E // UART CEA709.1-B Received Preamble Length
#define UART0_CPW		*(volatile uint8_t  *)0x4006A02F // UART CEA709.1-B Collision Pulse Width
#define UART0_RIDT		*(volatile uint8_t  *)0x4006A030 // UART CEA709.1-B Receive Indeterminate Time
#define UART0_TIDT		*(volatile uint8_t  *)0x4006A031 // UART CEA709.1-B Transmit Indeterminate Time
#define UART1_C7816		*(volatile uint8_t  *)0x4006B018 // UART 7816 Control Register
#define UART1_IE7816		*(volatile uint8_t  *)0x4006B019 // UART 7816 Interrupt Enable Register
#define UART1_IS7816		*(volatile uint8_t  *)0x4006B01A // UART 7816 Interrupt Status Register
#define UART1_WP7816T0		*(volatile uint8_t  *)0x4006B01B // UART 7816 Wait Parameter Register
#define UART1_WP7816T1		*(volatile uint8_t  *)0x4006B01B // UART 7816 Wait Parameter Register
#define UART1_WN7816		*(volatile uint8_t  *)0x4006B01C // UART 7816 Wait N Register
#define UART1_WF7816		*(volatile uint8_t  *)0x4006B01D // UART 7816 Wait FD Register
#define UART1_ET7816		*(volatile uint8_t  *)0x4006B01E // UART 7816 Error Threshold Register
#define UART1_TL7816		*(volatile uint8_t  *)0x4006B01F // UART 7816 Transmit Length Register
#define UART1_C6		*(volatile uint8_t  *)0x4006B021 // UART CEA709.1-B Control Register 6
#define UART1_PCTH		*(volatile uint8_t  *)0x4006B022 // UART CEA709.1-B Packet Cycle Time Counter High
#define UART1_PCTL		*(volatile uint8_t  *)0x4006B023 // UART CEA709.1-B Packet Cycle Time Counter Low
#define UART1_B1T		*(volatile uint8_t  *)0x4006B024 // UART CEA709.1-B Beta1 Timer
#define UART1_SDTH		*(volatile uint8_t  *)0x4006B025 // UART CEA709.1-B Secondary Delay Timer High
#define UART1_SDTL		*(volatile uint8_t  *)0x4006B026 // UART CEA709.1-B Secondary Delay Timer Low
#define UART1_PRE		*(volatile uint8_t  *)0x4006B027 // UART CEA709.1-B Preamble
#define UART1_TPL		*(volatile uint8_t  *)0x4006B028 // UART CEA709.1-B Transmit Packet Length
#define UART1_IE		*(volatile uint8_t  *)0x4006B029 // UART CEA709.1-B Interrupt Enable Register
#define UART1_WB		*(volatile uint8_t  *)0x4006B02A // UART CEA709.1-B WBASE
#define UART1_S3		*(volatile uint8_t  *)0x4006B02B // UART CEA709.1-B Status Register
#define UART1_S4		*(volatile uint8_t  *)0x4006B02C // UART CEA709.1-B Status Register
#define UART1_RPL		*(volatile uint8_t  *)0x4006B02D // UART CEA709.1-B Received Packet Length
#define UART1_RPREL		*(volatile uint8_t  *)0x4006B02E // UART CEA709.1-B Received Preamble Length
#define UART1_CPW		*(volatile uint8_t  *)0x4006B02F // UART CEA709.1-B Collision Pulse Width
#define UART1_RIDT		*(volatile uint8_t  *)0x4006B030 // UART CEA709.1-B Receive Indeterminate Time
#define UART1_TIDT		*(volatile uint8_t  *)0x4006B031 // UART CEA709.1-B Transmit Indeterminate Time
#define UART2_C7816		*(volatile uint8_t  *)0x4006C018 // UART 7816 Control Register
#define UART2_IE7816		*(volatile uint8_t  *)0x4006C019 // UART 7816 Interrupt Enable Register
#define UART2_IS7816		*(volatile uint8_t  *)0x4006C01A // UART 7816 Interrupt Status Register
#define UART2_WP7816T0		*(volatile uint8_t  *)0x4006C01B // UART 7816 Wait Parameter Register
#define UART2_WP7816T1		*(volatile uint8_t  *)0x4006C01B // UART 7816 Wait Parameter Register
#define UART2_WN7816		*(volatile uint8_t  *)0x4006C01C // UART 7816 Wait N Register
#define UART2_WF7816		*(volatile uint8_t  *)0x4006C01D // UART 7816 Wait FD Register
#define UART2_ET7816		*(volatile uint8_t  *)0x4006C01E // UART 7816 Error Threshold Register
#define UART2_TL7816		*(volatile uint8_t  *)0x4006C01F // UART 7816 Transmit Length Register
#define UART2_C6		*(volatile uint8_t  *)0x4006C021 // UART CEA709.1-B Control Register 6
#define UART2_PCTH		*(volatile uint8_t  *)0x4006C022 // UART CEA709.1-B Packet Cycle Time Counter High
#define UART2_PCTL		*(volatile uint8_t  *)0x4006C023 // UART CEA709.1-B Packet Cycle Time Counter Low
#define UART2_B1T		*(volatile uint8_t  *)0x4006C024 // UART CEA709.1-B Beta1 Timer
#define UART2_SDTH		*(volatile uint8_t  *)0x4006C025 // UART CEA709.1-B Secondary Delay Timer High
#define UART2_SDTL		*(volatile uint8_t  *)0x4006C026 // UART CEA709.1-B Secondary Delay Timer Low
#define UART2_PRE		*(volatile uint8_t  *)0x4006C027 // UART CEA709.1-B Preamble
#define UART2_TPL		*(volatile uint8_t  *)0x4006C028 // UART CEA709.1-B Transmit Packet Length
#define UART2_IE		*(volatile uint8_t  *)0x4006C029 // UART CEA709.1-B Interrupt Enable Register
#define UART2_WB		*(volatile uint8_t  *)0x4006C02A // UART CEA709.1-B WBASE
#define UART2_S3		*(volatile uint8_t  *)0x4006C02B // UART CEA709.1-B Status Register
#define UART2_S4		*(volatile uint8_t  *)0x4006C02C // UART CEA709.1-B Status Register
#define UART2_RPL		*(volatile uint8_t  *)0x4006C02D // UART CEA709.1-B Received Packet Length
#define UART2_RPREL		*(volatile uint8_t  *)0x4006C02E // UART CEA709.1-B Received Preamble Length
#define UART2_CPW		*(volatile uint8_t  *)0x4006C02F // UART CEA709.1-B Collision Pulse Width
#define UART2_RIDT		*(volatile uint8_t  *)0x4006C030 // UART CEA709.1-B Receive Indeterminate Time
#define UART2_TIDT		*(volatile uint8_t  *)0x4006C031 // UART CEA709.1-B Transmit Indeterminate Time

// Chapter 46: Synchronous Audio Interface (SAI)
#define I2S0_TCSR		*(volatile uint32_t *)0x4002F000 // SAI Transmit Control Register
#define I2S_TCSR_TE			(uint32_t)0x80000000	// Transmitter Enable
#define I2S_TCSR_STOPE			(uint32_t)0x40000000	// Transmitter Enable in Stop mode
#define I2S_TCSR_DBGE			(uint32_t)0x20000000	// Transmitter Enable in Debug mode
#define I2S_TCSR_BCE			(uint32_t)0x10000000	// Bit Clock Enable
#define I2S_TCSR_FR			(uint32_t)0x02000000	// FIFO Reset
#define I2S_TCSR_SR			(uint32_t)0x01000000	// Software Reset
#define I2S_TCSR_WSF			(uint32_t)0x00100000	// Word Start Flag
#define I2S_TCSR_SEF			(uint32_t)0x00080000	// Sync Error Flag
#define I2S_TCSR_FEF			(uint32_t)0x00040000	// FIFO Error Flag (underrun)
#define I2S_TCSR_FWF			(uint32_t)0x00020000	// FIFO Warning Flag (empty)
#define I2S_TCSR_FRF			(uint32_t)0x00010000	// FIFO Request Flag (Data Ready)
#define I2S_TCSR_WSIE			(uint32_t)0x00001000	// Word Start Interrupt Enable
#define I2S_TCSR_SEIE			(uint32_t)0x00000800	// Sync Error Interrupt Enable
#define I2S_TCSR_FEIE			(uint32_t)0x00000400	// FIFO Error Interrupt Enable
#define I2S_TCSR_FWIE			(uint32_t)0x00000200	// FIFO Warning Interrupt Enable
#define I2S_TCSR_FRIE			(uint32_t)0x00000100	// FIFO Request Interrupt Enable
#define I2S_TCSR_FWDE			(uint32_t)0x00000002	// FIFO Warning DMA Enable
#define I2S_TCSR_FRDE			(uint32_t)0x00000001	// FIFO Request DMA Enable
#define I2S0_TCR1		*(volatile uint32_t *)0x4002F004 // SAI Transmit Configuration 1 Register
#define I2S_TCR1_TFW(n)			((uint32_t)n & 0x03)	      // Transmit FIFO watermark
#define I2S0_TCR2		*(volatile uint32_t *)0x4002F008 // SAI Transmit Configuration 2 Register
#define I2S_TCR2_DIV(n)			((uint32_t)n & 0xff)	      // Bit clock divide by (DIV+1)*2
#define I2S_TCR2_BCD			((uint32_t)1<<24)	      // Bit clock direction
#define I2S_TCR2_BCP			((uint32_t)1<<25)	      // Bit clock polarity
#define I2S_TCR2_MSEL(n)		((uint32_t)(n & 3)<<26)	      // MCLK select, 0=bus clock, 1=I2S0_MCLK
#define I2S_TCR2_BCI			((uint32_t)1<<28)	      // Bit clock input
#define I2S_TCR2_BCS			((uint32_t)1<<29)	      // Bit clock swap
#define I2S_TCR2_SYNC(n)		((uint32_t)(n & 3)<<30)	      // 0=async 1=sync with receiver
#define I2S0_TCR3		*(volatile uint32_t *)0x4002F00C // SAI Transmit Configuration 3 Register
#define I2S_TCR3_WDFL(n)		((uint32_t)n & 0x0f)	      // word flag configuration
#define I2S_TCR3_TCE			((uint32_t)0x10000)	      // transmit channel enable
#define I2S0_TCR4		*(volatile uint32_t *)0x4002F010 // SAI Transmit Configuration 4 Register
#define I2S_TCR4_FSD			((uint32_t)1)		      // Frame Sync Direction
#define I2S_TCR4_FSP			((uint32_t)2)		      // Frame Sync Polarity
#define I2S_TCR4_FSE			((uint32_t)8)		      // Frame Sync Early
#define I2S_TCR4_MF			((uint32_t)0x10)	      // MSB First
#define I2S_TCR4_SYWD(n)		((uint32_t)(n & 0x1f)<<8)     // Sync Width
#define I2S_TCR4_FRSZ(n)		((uint32_t)(n & 0x0f)<<16)    // Frame Size
#define I2S0_TCR5		*(volatile uint32_t *)0x4002F014 // SAI Transmit Configuration 5 Register
#define I2S_TCR5_FBT(n)			((uint32_t)(n & 0x1f)<<8)     // First Bit Shifted
#define I2S_TCR5_W0W(n)			((uint32_t)(n & 0x1f)<<16)    // Word 0 Width
#define I2S_TCR5_WNW(n)			((uint32_t)(n & 0x1f)<<24)    // Word N Width
#define I2S0_TDR0		*(volatile uint32_t *)0x4002F020 // SAI Transmit Data Register
#define I2S0_TDR1		*(volatile uint32_t *)0x4002F024 // SAI Transmit Data Register
#define I2S0_TFR0		*(volatile uint32_t *)0x4002F040 // SAI Transmit FIFO Register
#define I2S0_TFR1		*(volatile uint32_t *)0x4002F044 // SAI Transmit FIFO Register
#define I2S_TFR_RFP(n)			((uint32_t)n & 7)	      // read FIFO pointer
#define I2S_TFR_WFP(n)			((uint32_t)(n & 7)<<16)	      // write FIFO pointer
#define I2S0_TMR		*(volatile uint32_t *)0x4002F060 // SAI Transmit Mask Register
#define I2S_TMR_TWM(n)			((uint32_t)n & 0xFFFFFFFF)
#define I2S0_RCSR		*(volatile uint32_t *)0x4002F080 // SAI Receive Control Register
#define I2S_RCSR_RE			(uint32_t)0x80000000	// Receiver Enable
#define I2S_RCSR_STOPE			(uint32_t)0x40000000	// Receiver Enable in Stop mode
#define I2S_RCSR_DBGE			(uint32_t)0x20000000	// Receiver Enable in Debug mode
#define I2S_RCSR_BCE			(uint32_t)0x10000000	// Bit Clock Enable
#define I2S_RCSR_FR			(uint32_t)0x02000000	// FIFO Reset
#define I2S_RCSR_SR			(uint32_t)0x01000000	// Software Reset
#define I2S_RCSR_WSF			(uint32_t)0x00100000	// Word Start Flag
#define I2S_RCSR_SEF			(uint32_t)0x00080000	// Sync Error Flag
#define I2S_RCSR_FEF			(uint32_t)0x00040000	// FIFO Error Flag (underrun)
#define I2S_RCSR_FWF			(uint32_t)0x00020000	// FIFO Warning Flag (empty)
#define I2S_RCSR_FRF			(uint32_t)0x00010000	// FIFO Request Flag (Data Ready)
#define I2S_RCSR_WSIE			(uint32_t)0x00001000	// Word Start Interrupt Enable
#define I2S_RCSR_SEIE			(uint32_t)0x00000800	// Sync Error Interrupt Enable
#define I2S_RCSR_FEIE			(uint32_t)0x00000400	// FIFO Error Interrupt Enable
#define I2S_RCSR_FWIE			(uint32_t)0x00000200	// FIFO Warning Interrupt Enable
#define I2S_RCSR_FRIE			(uint32_t)0x00000100	// FIFO Request Interrupt Enable
#define I2S_RCSR_FWDE			(uint32_t)0x00000002	// FIFO Warning DMA Enable
#define I2S_RCSR_FRDE			(uint32_t)0x00000001	// FIFO Request DMA Enable
#define I2S0_RCR1		*(volatile uint32_t *)0x4002F084 // SAI Receive Configuration 1 Register
#define I2S_RCR1_RFW(n)			((uint32_t)n & 0x03)	      // Receive FIFO watermark
#define I2S0_RCR2		*(volatile uint32_t *)0x4002F088 // SAI Receive Configuration 2 Register
#define I2S_RCR2_DIV(n)			((uint32_t)n & 0xff)	      // Bit clock divide by (DIV+1)*2
#define I2S_RCR2_BCD			((uint32_t)1<<24)	      // Bit clock direction
#define I2S_RCR2_BCP			((uint32_t)1<<25)	      // Bit clock polarity
#define I2S_RCR2_MSEL(n)		((uint32_t)(n & 3)<<26)	      // MCLK select, 0=bus clock, 1=I2S0_MCLK
#define I2S_RCR2_BCI			((uint32_t)1<<28)	      // Bit clock input
#define I2S_RCR2_BCS			((uint32_t)1<<29)	      // Bit clock swap
#define I2S_RCR2_SYNC(n)		((uint32_t)(n & 3)<<30)	      // 0=async 1=sync with receiver
#define I2S0_RCR3		*(volatile uint32_t *)0x4002F08C // SAI Receive Configuration 3 Register
#define I2S_RCR3_WDFL(n)		((uint32_t)n & 0x0f)	      // word flag configuration
#define I2S_RCR3_RCE			((uint32_t)0x10000)	      // receive channel enable
#define I2S0_RCR4		*(volatile uint32_t *)0x4002F090 // SAI Receive Configuration 4 Register
#define I2S_RCR4_FSD			((uint32_t)1)		      // Frame Sync Direction
#define I2S_RCR4_FSP			((uint32_t)2)		      // Frame Sync Polarity
#define I2S_RCR4_FSE			((uint32_t)8)		      // Frame Sync Early
#define I2S_RCR4_MF			((uint32_t)0x10)	      // MSB First
#define I2S_RCR4_SYWD(n)		((uint32_t)(n & 0x1f)<<8)     // Sync Width
#define I2S_RCR4_FRSZ(n)		((uint32_t)(n & 0x0f)<<16)    // Frame Size
#define I2S0_RCR5		*(volatile uint32_t *)0x4002F094 // SAI Receive Configuration 5 Register
#define I2S_RCR5_FBT(n)			((uint32_t)(n & 0x1f)<<8)     // First Bit Shifted
#define I2S_RCR5_W0W(n)			((uint32_t)(n & 0x1f)<<16)    // Word 0 Width
#define I2S_RCR5_WNW(n)			((uint32_t)(n & 0x1f)<<24)    // Word N Width
#define I2S0_RDR0		*(volatile uint32_t *)0x4002F0A0 // SAI Receive Data Register
#define I2S0_RDR1		*(volatile uint32_t *)0x4002F0A4 // SAI Receive Data Register
#define I2S0_RFR0		*(volatile uint32_t *)0x4002F0C0 // SAI Receive FIFO Register
#define I2S0_RFR1		*(volatile uint32_t *)0x4002F0C4 // SAI Receive FIFO Register
#define I2S_RFR_RFP(n)			((uint32_t)n & 7)	      // read FIFO pointer
#define I2S_RFR_WFP(n)			((uint32_t)(n & 7)<<16)	      // write FIFO pointer
#define I2S0_RMR		*(volatile uint32_t *)0x4002F0E0 // SAI Receive Mask Register
#define I2S_RMR_RWM(n)			((uint32_t)n & 0xFFFFFFFF)
#define I2S0_MCR		*(volatile uint32_t *)0x4002F100 // SAI MCLK Control Register
#define I2S_MCR_DUF			((uint32_t)1<<31)	      // Divider Update Flag
#define I2S_MCR_MOE			((uint32_t)1<<30)	      // MCLK Output Enable
#define I2S_MCR_MICS(n)			((uint32_t)(n & 3)<<24)	      // MCLK Input Clock Select
#define I2S0_MDR		*(volatile uint32_t *)0x4002F104 // SAI MCLK Divide Register
#define I2S_MDR_FRACT(n)		((uint32_t)(n & 0xff)<<12)    // MCLK Fraction
#define I2S_MDR_DIVIDE(n)		((uint32_t)(n & 0xfff))	      // MCLK Divide

// Chapter 47: General-Purpose Input/Output (GPIO)

// Chapter 48: Touch sense input (TSI)
#define TSI_GENCS_LPCLKS		(uint32_t)0x10000000		// 
#define TSI_GENCS_EOSF			(uint32_t)0x00008000		// 
#define TSI_GENCS_OUTRGF		(uint32_t)0x00004000		// 
#define TSI_GENCS_EXTERF		(uint32_t)0x00002000		// 
#define TSI_GENCS_OVRF			(uint32_t)0x00001000		// 
#define TSI_GENCS_SCNIP			(uint32_t)0x00000200		// 
#define TSI_GENCS_SWTS			(uint32_t)0x00000100		// 
#define TSI_GENCS_TSIEN			(uint32_t)0x00000080		// 
#define TSI_GENCS_TSIIE			(uint32_t)0x00000040		// 
#define TSI_GENCS_ERIE			(uint32_t)0x00000020		// 
#define TSI_GENCS_ESOR			(uint32_t)0x00000010		// 
#define TSI_GENCS_STM			(uint32_t)0x00000002		// 
#define TSI_GENCS_STPE			(uint32_t)0x00000001		// 
#define TSI0_WUCNTR		*(volatile uint32_t *)0x4004500C // Wake-Up Channel Counter Register
#define TSI0_THRESHOLD		*(volatile uint32_t *)0x40045120 // Low Power Channel Threshold Register

// Nested Vectored Interrupt Controller, Table 3-4 & ARMv7 ref, appendix B3.4 (page 750)
#define NVIC_ENABLE_IRQ(n)	(*((volatile uint32_t *)0xE000E100 + (n >> 5)) = (1 << (n & 31)))
#define NVIC_DISABLE_IRQ(n)	(*((volatile uint32_t *)0xE000E180 + (n >> 5)) = (1 << (n & 31)))
#define NVIC_SET_PENDING(n)	(*((volatile uint32_t *)0xE000E200 + (n >> 5)) = (1 << (n & 31)))
#define NVIC_CLEAR_PENDING(n)	(*((volatile uint32_t *)0xE000E280 + (n >> 5)) = (1 << (n & 31)))

#define NVIC_ISER0		*(volatile uint32_t *)0xE000E100
#define NVIC_ISER1		*(volatile uint32_t *)0xE000E104
#define NVIC_ICER0		*(volatile uint32_t *)0xE000E180
#define NVIC_ICER1		*(volatile uint32_t *)0xE000E184

// 0 = highest priority
// Cortex-M4: 0,16,32,48,64,80,96,112,128,144,160,176,192,208,224,240
// Cortex-M0: 0,64,128,192
#define NVIC_SET_PRIORITY(irqnum, priority)  (*((volatile uint8_t *)0xE000E400 + (irqnum)) = (uint8_t)(priority))
#define NVIC_GET_PRIORITY(irqnum) (*((uint8_t *)0xE000E400 + (irqnum)))

#if defined(__MK20DX128__)
#define IRQ_DMA_CH0		0
#define IRQ_DMA_CH1		1
#define IRQ_DMA_CH2		2
#define IRQ_DMA_CH3		3
#define IRQ_DMA_ERROR		4
#define IRQ_FTFL_COMPLETE	6
#define IRQ_FTFL_COLLISION	7
#define IRQ_LOW_VOLTAGE		8
#define IRQ_LLWU		9
#define IRQ_WDOG		10
#define IRQ_I2C0		11
#define IRQ_SPI0		12
#define IRQ_I2S0_TX		13
#define IRQ_I2S0_RX		14
#define IRQ_UART0_LON		15
#define IRQ_UART0_STATUS	16
#define IRQ_UART0_ERROR		17
#define IRQ_UART1_STATUS	18
#define IRQ_UART1_ERROR		19
#define IRQ_UART2_STATUS	20
#define IRQ_UART2_ERROR		21
#define IRQ_ADC0		22
#define IRQ_CMP0		23
#define IRQ_CMP1		24
#define IRQ_FTM0		25
#define IRQ_FTM1		26
#define IRQ_CMT			27
#define IRQ_RTC_ALARM		28
#define IRQ_RTC_SECOND		29
#define IRQ_PIT_CH0		30
#define IRQ_PIT_CH1		31
#define IRQ_PIT_CH2		32
#define IRQ_PIT_CH3		33
#define IRQ_PDB			34
#define IRQ_USBOTG		35
#define IRQ_USBDCD		36
#define IRQ_TSI			37
#define IRQ_MCG			38
#define IRQ_LPTMR		39
#define IRQ_PORTA		40
#define IRQ_PORTB		41
#define IRQ_PORTC		42
#define IRQ_PORTD		43
#define IRQ_PORTE		44
#define IRQ_SOFTWARE		45
#define NVIC_NUM_INTERRUPTS	46

#elif defined(__MK20DX256__)
#define IRQ_DMA_CH0		0
#define IRQ_DMA_CH1		1
#define IRQ_DMA_CH2		2
#define IRQ_DMA_CH3		3
#define IRQ_DMA_CH4		4
#define IRQ_DMA_CH5		5
#define IRQ_DMA_CH6		6
#define IRQ_DMA_CH7		7
#define IRQ_DMA_CH8		8
#define IRQ_DMA_CH9		9
#define IRQ_DMA_CH10		10
#define IRQ_DMA_CH11		11
#define IRQ_DMA_CH12		12
#define IRQ_DMA_CH13		13
#define IRQ_DMA_CH14		14
#define IRQ_DMA_CH15		15
#define IRQ_DMA_ERROR		16
#define IRQ_FTFL_COMPLETE	18
#define IRQ_FTFL_COLLISION	19
#define IRQ_LOW_VOLTAGE		20
#define IRQ_LLWU		21
#define IRQ_WDOG		22
#define IRQ_I2C0		24
#define IRQ_I2C1		25
#define IRQ_SPI0		26
#define IRQ_SPI1		27
#define IRQ_CAN_MESSAGE		29
#define IRQ_CAN_BUS_OFF		30
#define IRQ_CAN_ERROR		31
#define IRQ_CAN_TX_WARN		32
#define IRQ_CAN_RX_WARN		33
#define IRQ_CAN_WAKEUP		34
#define IRQ_I2S0_TX		35
#define IRQ_I2S0_RX		36
#define IRQ_UART0_LON		44
#define IRQ_UART0_STATUS	45
#define IRQ_UART0_ERROR		46
#define IRQ_UART1_STATUS	47
#define IRQ_UART1_ERROR		48
#define IRQ_UART2_STATUS	49
#define IRQ_UART2_ERROR		50
#define IRQ_ADC0		57
#define IRQ_ADC1		58
#define IRQ_CMP0		59
#define IRQ_CMP1		60
#define IRQ_CMP2		61
#define IRQ_FTM0		62
#define IRQ_FTM1		63
#define IRQ_FTM2		64
#define IRQ_CMT			65
#define IRQ_RTC_ALARM		66
#define IRQ_RTC_SECOND		67
#define IRQ_PIT_CH0		68
#define IRQ_PIT_CH1		69
#define IRQ_PIT_CH2		70
#define IRQ_PIT_CH3		71
#define IRQ_PDB			72
#define IRQ_USBOTG		73
#define IRQ_USBDCD		74
#define IRQ_DAC0		81
#define IRQ_TSI			83
#define IRQ_MCG			84
#define IRQ_LPTMR		85
#define IRQ_PORTA		87
#define IRQ_PORTB		88
#define IRQ_PORTC		89
#define IRQ_PORTD		90
#define IRQ_PORTE		91
#define IRQ_SOFTWARE		94
#define NVIC_NUM_INTERRUPTS	95

#endif





#define __disable_irq() asm volatile("CPSID i");
#define __enable_irq()	asm volatile("CPSIE i");

// System Control Space (SCS), ARMv7 ref manual, B3.2, page 708
#define SCB_ICSR_PENDSTSET		(uint32_t)0x04000000

#define SYST_CSR_COUNTFLAG		(uint32_t)0x00010000
#define SYST_CSR_CLKSOURCE		(uint32_t)0x00000004
#define SYST_CSR_TICKINT		(uint32_t)0x00000002
#define SYST_CSR_ENABLE			(uint32_t)0x00000001


#define ARM_DEMCR		*(volatile uint32_t *)0xE000EDFC // Debug Exception and Monitor Control
#define ARM_DEMCR_TRCENA		(1 << 24)	 // Enable debugging & monitoring blocks
#define ARM_DWT_CTRL		*(volatile uint32_t *)0xE0001000 // DWT control register
#define ARM_DWT_CTRL_CYCCNTENA		(1 << 0)		// Enable cycle count
#define ARM_DWT_CYCCNT		*(volatile uint32_t *)0xE0001004 // Cycle count register

extern int nvic_execution_priority(void);

extern void nmi_isr(void);
extern void hard_fault_isr(void);
extern void memmanage_fault_isr(void);
extern void bus_fault_isr(void);
extern void usage_fault_isr(void);
extern void svcall_isr(void);
extern void debugmonitor_isr(void);
extern void pendablesrvreq_isr(void);
extern void systick_isr(void);
extern void dma_ch0_isr(void);
extern void dma_ch1_isr(void);
extern void dma_ch2_isr(void);
extern void dma_ch3_isr(void);
extern void dma_ch4_isr(void);
extern void dma_ch5_isr(void);
extern void dma_ch6_isr(void);
extern void dma_ch7_isr(void);
extern void dma_ch8_isr(void);
extern void dma_ch9_isr(void);
extern void dma_ch10_isr(void);
extern void dma_ch11_isr(void);
extern void dma_ch12_isr(void);
extern void dma_ch13_isr(void);
extern void dma_ch14_isr(void);
extern void dma_ch15_isr(void);
extern void dma_error_isr(void);
extern void mcm_isr(void);
extern void flash_cmd_isr(void);
extern void flash_error_isr(void);
extern void low_voltage_isr(void);
extern void wakeup_isr(void);
extern void watchdog_isr(void);
extern void i2c0_isr(void);
extern void i2c1_isr(void);
extern void i2c2_isr(void);
extern void spi0_isr(void);
extern void spi1_isr(void);
extern void spi2_isr(void);
extern void sdhc_isr(void);
extern void can0_message_isr(void);
extern void can0_bus_off_isr(void);
extern void can0_error_isr(void);
extern void can0_tx_warn_isr(void);
extern void can0_rx_warn_isr(void);
extern void can0_wakeup_isr(void);
extern void i2s0_tx_isr(void);
extern void i2s0_rx_isr(void);
extern void uart0_lon_isr(void);
extern void uart0_status_isr(void);
extern void uart0_error_isr(void);
extern void uart1_status_isr(void);
extern void uart1_error_isr(void);
extern void uart2_status_isr(void);
extern void uart2_error_isr(void);
extern void uart3_status_isr(void);
extern void uart3_error_isr(void);
extern void uart4_status_isr(void);
extern void uart4_error_isr(void);
extern void uart5_status_isr(void);
extern void uart5_error_isr(void);
extern void adc0_isr(void);
extern void adc1_isr(void);
extern void cmp0_isr(void);
extern void cmp1_isr(void);
extern void cmp2_isr(void);
extern void ftm0_isr(void);
extern void ftm1_isr(void);
extern void ftm2_isr(void);
extern void ftm3_isr(void);
extern void cmt_isr(void);
extern void rtc_alarm_isr(void);
extern void rtc_seconds_isr(void);
extern void pit0_isr(void);
extern void pit1_isr(void);
extern void pit2_isr(void);
extern void pit3_isr(void);
extern void pdb_isr(void);
extern void usb_isr(void);
extern void usb_charge_isr(void);
extern void dac0_isr(void);
extern void dac1_isr(void);
extern void tsi0_isr(void);
extern void mcg_isr(void);
extern void lptmr_isr(void);
extern void porta_isr(void);
extern void portb_isr(void);
extern void portc_isr(void);
extern void portd_isr(void);
extern void porte_isr(void);
extern void software_isr(void);



#ifdef __cplusplus
}
#endif
#endif
