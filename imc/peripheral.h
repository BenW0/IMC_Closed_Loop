#ifndef peripheral_h
#define peripheral_h

#include <stdint.h>
#include <mk20dx128.h>
#include <pin_config.h>

// Pins 2,3,4,5
#define ADDR_BIT0_PORT(reg) GPIOD_P##reg
#define ADDR_BIT1_PORT(reg) GPIOA_P##reg
#define ADDR_BIT2_PORT(reg) GPIOA_P##reg
#define ADDR_BIT3_PORT(reg) GPIOD_P##reg

#define ADDR_BIT0_CTRL  PORTD_PCR0
#define ADDR_BIT0_BIT   1

#define ADDR_BIT1_CTRL  PORTA_PCR12
#define ADDR_BIT1_BIT   (1<<12)

#define ADDR_BIT2_CTRL  PORTA_PCR13
#define ADDR_BIT2_BIT   (1<<13)

#define ADDR_BIT3_CTRL  PORTD_PCR7
#define ADDR_BIT3_BIT   (1<<7)

uint8_t read_i2c_address(void);

// Pins 6,7,8

#define MOTOR_BIT_PORT(reg) GPIOD_P##reg

#define MOTOR_BIT0_CTRL       PORTD_PCR4
#define MOTOR_BIT0_BIT        (1<<4)

#define MOTOR_BIT1_CTRL       PORTD_PCR2
#define MOTOR_BIT1_BIT        (1<<2)

#define MOTOR_BIT2_CTRL       PORTD_PCR3
#define MOTOR_BIT2_BIT        (1<<3)

#define MOTOR_BIT_MASK  (MOTOR_BIT0_BIT | MOTOR_BIT1_BIT | MOTOR_BIT2_BIT)

// Microstepping settings for the A4988 stepper driver module
#define MAX_MICROSTEP 16
#define STEP_TABLE {0, 1, 2, 3, 7} // Indexed by base-two log of microstepping setting

// Microstepping settings for DRV8825 stepper driver module
// #define MAX_MICROSTEP 32
// #define STEP_TABLE {0, 1, 2, 3, 4, 5}

uint32_t set_microstepping(uint32_t);

#endif
