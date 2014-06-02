#include "hardware.h"
#include "parameters.h"
#include <mk20dx128.h>
#include <pin_config.h>

#define FLIP_AXIS (1<<0)
#define HOME_DIR  (1<<1)
#define ENABLE_MIN (1<<4)
#define ENABLE_MAX (1<<5)
#define INVERT_MIN (1<<6)
#define INVERT_MAX (1<<7)

// Set the data-direction, multiplexing, and pullup/down/high-z for a limit
// pin. Also enables interrupts.
void configure_limit_gpio(uint32_t axis, imc_pullup_values value, uint32_t homing){
  uint32_t config = MUX_GPIO;
  uint32_t enable = axis ? ENABLE_MAX : ENABLE_MIN;
  uint32_t invert = axis ? INVERT_MAX : INVERT_MIN;

  if(enable & homing){
    if(invert & homing)
      config |= IRQC_FALLING;
    else
      config |= IRQC_RISING;
    // Otherwise, no irq
  }

  if(value == PRESERVE_PULLUP){
    config |= (axis ? MAX_LIMIT_CTRL : MIN_LIMIT_CTRL) & ( PULL_DOWN | PULL_UP); 
  }else if(value != IMC_NO_PULL){
    config |= (value == IMC_PULLDOWN) ? PULL_DOWN : PULL_UP;
  }
  
  if(axis){
    // Set the pin as output in the port ddr
    CONTROL_DDR &= ~MAX_LIMIT_BIT;
    // Set the correct pin config register
    MAX_LIMIT_CTRL = config;
  }else{
    CONTROL_DDR &= ~MIN_LIMIT_BIT;
    MIN_LIMIT_CTRL = config; 
  }
}

// This is dependent on the parameters structure being properly configured
// before execution
void reset_hardware(void){
  STEPPER_DDR |= DISABLE_BIT | DIR_BIT | STEP_BIT;
  DISABLE_CTRL = STANDARD_OUTPUT;
  DIR_CTRL = STANDARD_OUTPUT;
  STEP_CTRL = STANDARD_OUTPUT;
  // Put stepper driver in a safe condition - todo: allow for inverting disable and step
  STEPPER_PORT(SOR) = DISABLE_BIT;
  STEPPER_PORT(COR) = STEP_BIT;

  // As much as I don't like leaving the pin floating, I have no idea what this should default to.
  configure_limit_gpio(0, IMC_PULLDOWN, parameters.homing);
  configure_limit_gpio(1, IMC_PULLDOWN, parameters.homing);
}

// this function has been modified to prioritize all interrupts for the
// entire project, not just the imc modules.
void configure_nvic(void){
  // Start in the idle state, but first wake up to check for keep steppers enabled option.
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH2);

  // This is already done in the default boot sequence, but that could change and enabling is idempotent...
  // port b is responsible for limits and the sync protocol - rather critical to have interrupts.
  NVIC_ENABLE_IRQ(IRQ_PORTB);
  NVIC_ENABLE_IRQ(IRQ_I2C0);

  // reset the prioritization of interrupts on our chip. We'll set everything down to 3, then
  // boost up the control interrupt to 2 AND the stepper interrupts to 1. 
  // IRQ priorities are set using the high nibble.
  for(uint32_t i = 0; i < NVIC_NUM_INTERRUPTS; i++)
  {
    NVIC_SET_PRIORITY(i, 3<<4);
  }
  // control ISR
  NVIC_SET_PRIORITY(IRQ_PIT_CH3, 2<<4);       // this priority level is explicity used in spienc.c:read_spi()

  // Limits/sync and pin toggle/reset isr get the highest priority
  NVIC_SET_PRIORITY(IRQ_PORTB, 0);
  NVIC_SET_PRIORITY(IRQ_PIT_CH1, 1<<4);
  // Followed by the main stepper isr
  NVIC_SET_PRIORITY(IRQ_PIT_CH0, 1<<4);
  // Lastly, the sync reset/timeout isr and i2c communication
  NVIC_SET_PRIORITY(IRQ_I2C0, 2<<4);
  NVIC_SET_PRIORITY(IRQ_PIT_CH2, 2<<4);
}

