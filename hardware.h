#ifndef hardware_h
#include <mk20dx128.h>
#define hardware_h
// All stepper control is placed on port c
#define STEPPER_PORT(reg) GPIOC_P##reg
#define STEPPER_DDR GPIOC_PDDR
// Pin 9  is stepper disable
// Pin 10 is direction
// Pin 11 is step
#define DISABLE_CTRL PORTC_PCR3
#define DISABLE_BIT  (1<<3)
#define DIR_CTRL PORTC_PCR4
#define DIR_BIT  (1<<4)
#define STEP_CTRL PORTC_PCR5
#define STEP_BIT  (1<<5)

// All input is placed on port b
// Pin 16 is the global sync line
// Pin 0 is the axis minimum limit switch
// Pin 1 is the axis maximum limit switch
// Pin 32 is quadrature input channel A
// Pin 25 is quadrature input channel B
#define CONTROL_PORT(reg) GPIOB_P##reg
#define CONTROL_DDR       GPIOB_PDDR

#define SYNC_CTRL PORTB_PCR0
#define SYNC_BIT  1

#define MIN_LIMIT_CTRL PORTB_PCR16
#define MIN_LIMIT_BIT  (1<<16)
#define MAX_LIMIT_CTRL PORTB_PCR17
#define MAX_LIMIT_BIT  (1<<17)
#define LIMIT_MASK (MAX_LIMIT_BIT | MIN_LIMIT_BIT)

#define I2C_PORT(reg) GPIOB_P##reg
#define I2C_DDR       GPIOB_PDDR

#define SDA_CTRL PORTB_PCR3
#define SDA_BIT  (1<<3)

#define SCL_CTRL PORTB_PCR2
#define SCL_BIT  (1<<2)

// Bits in the PIT register:
#define TIE 2 // Timer interrupt enable
#define TEN 1 // Timer enable

void reset_hardware(void);
// Set the data-direction, multiplexing, and pullup/down/high-z for a limit
// pin. Also enables interrupts.
//void configure_limit_gpio(uint32_t, imc_pullup_values, uint32_t);
#endif
