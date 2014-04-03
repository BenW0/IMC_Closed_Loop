#include "common.h"
#include "hardware.h"
#include <pin_config.h>

#include "hardware.h"

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
}
