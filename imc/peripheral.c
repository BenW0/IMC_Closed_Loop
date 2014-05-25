#include "peripheral.h"

#include <stdint.h>
#include <mk20dx128.h>
#include <pin_config.h>
#include <util.h>

uint8_t read_i2c_address(void){
  uint32_t config = MUX_GPIO | PULL_UP;
  uint8_t addr = 0;

  ADDR_BIT0_CTRL  = config;
  ADDR_BIT1_CTRL  = config;
  ADDR_BIT2_CTRL  = config;
  ADDR_BIT3_CTRL  = config;
  
  ADDR_BIT0_PORT(DDR) &= ~ADDR_BIT0_BIT;
  ADDR_BIT1_PORT(DDR) &= ~ADDR_BIT1_BIT;
  ADDR_BIT2_PORT(DDR) &= ~ADDR_BIT2_BIT;
  ADDR_BIT3_PORT(DDR) &= ~ADDR_BIT3_BIT;
  
  delay_microseconds(1); // Probably not needed, but wait for things to stabilize

  addr |=  (ADDR_BIT0_PORT(DIR) & ADDR_BIT0_BIT) ? 0 : 1;
  addr |=  (ADDR_BIT1_PORT(DIR) & ADDR_BIT1_BIT) ? 0 : 2;
  addr |=  (ADDR_BIT2_PORT(DIR) & ADDR_BIT2_BIT) ? 0 : 4;
  addr |=  (ADDR_BIT3_PORT(DIR) & ADDR_BIT3_BIT) ? 0 : 8;
  
  config = MUX_GPIO | PULL_NONE;
  ADDR_BIT0_CTRL  = config;
  ADDR_BIT1_CTRL  = config;
  ADDR_BIT2_CTRL  = config;
  ADDR_BIT3_CTRL  = config;
  
  return addr;
}

uint32_t set_microstepping(uint32_t steps){
  uint32_t table[] = STEP_TABLE;
  uint32_t log = 0;
  if(steps == 0 || steps > MAX_MICROSTEP) return 0;
  // Steps must have at least one set bit, so this must terminate
  while(!(steps & 1)){
    steps = steps >> 1;
    log++;
  }
  if(steps & ~1) return 0; // Had other bits set, not a power of two
  log =  table[log];

  MOTOR_BIT0_CTRL = STANDARD_OUTPUT;
  MOTOR_BIT1_CTRL = STANDARD_OUTPUT;
  MOTOR_BIT2_CTRL = STANDARD_OUTPUT;

  MOTOR_BIT_PORT(DDR) = MOTOR_BIT0_BIT | MOTOR_BIT1_BIT | MOTOR_BIT2_BIT;

  log = ((log & 1) ? MOTOR_BIT0_BIT : 0) | ((log & 2) ? MOTOR_BIT1_BIT : 0) | ((log & 4) ? MOTOR_BIT2_BIT : 0);
 
  MOTOR_BIT_PORT(DOR) = (MOTOR_BIT_MASK & MOTOR_BIT_PORT(DOR)) | log;

  return 1;
}
