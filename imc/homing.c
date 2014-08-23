/*
 * This software is (c) 2014 by Ben Weiss and is released under the following license:
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Matthew D Sorensen and Ben Weiss
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "hardware.h"
#include "control_isr.h"
#include "homing.h"
#include "stepper.h"
#include "parameters.h"
#include "config.h"

#include <util.h>

#define MICROS_PER_MINUTE 60000000L
#define PULSE_LENGTH 2 // General pulse length / propagation delay for all stepper signals
#define MICROS_PER_TICK (1000000L / ACCELERATION_TICKS_PER_SECOND)

bool (*homing_start_hook_fun)(void) = NULL;
void (*homing_end_hook_fun)(void) = NULL;

// Give the host module access to override the homing routine, or make appropriate changes to 
// ensure any higher-level controller doesn't freak out.
void set_homing_hooks(bool (*start_hook)(), void (*end_hook)())
{
  homing_start_hook_fun = start_hook;
  homing_end_hook_fun = end_hook;
}

static int32_t untrigger_limit(uint32_t mask, uint32_t invert_mask, uint32_t speed){
  int32_t steps = 0;
  while((CONTROL_PORT(DIR) & mask) ^ invert_mask){
    STEPPER_PORT(SOR) = STEP_BIT;
    delay_microseconds(PULSE_LENGTH);
    STEPPER_PORT(COR) = STEP_BIT;
    steps++;
    delay_microseconds(speed);
  }
  return steps;
}

static int32_t trigger_limit(uint32_t mask, uint32_t invert_mask, uint32_t speed){
  int32_t steps = 0;
  while(!((CONTROL_PORT(DIR) & mask) ^ invert_mask)){
    STEPPER_PORT(SOR) = STEP_BIT;
    delay_microseconds(PULSE_LENGTH);
    STEPPER_PORT(COR) = STEP_BIT;
    steps++;
    delay_microseconds(speed);
  }
  return steps;
}
static void take_steps(uint32_t steps, uint32_t speed){
  while(steps-- > 0){
    STEPPER_PORT(SOR) = STEP_BIT;
    delay_microseconds(PULSE_LENGTH);
    STEPPER_PORT(COR) = STEP_BIT;
    delay_microseconds(speed);
  }
}

// name changed to avoid conflicts with the new stepper.c:set_direction() method.
static void set_homing_direction(uint32_t dir){
  STEPPER_PORT(DOR) = (STEPPER_PORT(DOR) & ~DIR_BIT) | (dir ? DIR_BIT : 0);
  delay_microseconds(PULSE_LENGTH);
}

void enter_homing_routine(void){
  uint32_t homing, direction_bit;
  uint32_t inv_homing_feed, backoff;
  uint32_t mask, invert_mask;
  int32_t steps;
  // If we're not in idle mode, hard error
  if(st.state != STATE_IDLE){
    st.state = STATE_ERROR; // Probably should set a real error code
    return;
  }

  // notify the host module we are in the homing routine.
  if(homing_start_hook_fun)
    if(homing_start_hook_fun())
    {
      if(homing_end_hook_fun)
        homing_end_hook_fun();
      return;   // the hook dealt with it.
    }
  

  homing = parameters.homing;
  // Disable the appropriate hard limit function - choose the side we're homing to, don't change the pull-up
  // state, and pass a homing bit mask that results in either direction disabled.
  configure_limit_gpio(homing & HOME_DIR, PRESERVE_PULLUP, ~ (ENABLE_MIN | ENABLE_MAX));
  // Make sure our motor is enabled 
  enable_stepper();
  // If the flip bit is asserted, we should home to the opposite of the home direction bit
  direction_bit = !((homing & FLIP_AXIS) ^ ((homing & HOME_DIR) >> 1));   // I think this not being inverted was a bug.
  set_homing_direction(direction_bit);

  inv_homing_feed = MICROS_PER_MINUTE / parameters.homing_feedrate;
  backoff = HOMING_BACKOFF * 1000 / inv_homing_feed;

  // Compute the masks to use in determining our end condition
  if(homing & HOME_DIR){
    mask = MAX_LIMIT_BIT;
    invert_mask = (homing & INVERT_MAX) ? mask : 0;
  }else{
    mask = MIN_LIMIT_BIT;
    invert_mask = (homing & INVERT_MIN) ? mask : 0;
  }
  // Execute the first homing pass at the nominal homing speed
  steps = trigger_limit(mask, invert_mask, inv_homing_feed);
  // Back off at minimum velocity until the switch is deasserted, taking into account the negative distance
  set_homing_direction(!direction_bit);
  steps -= untrigger_limit(mask, invert_mask, MICROS_PER_MINUTE / MINIMUM_STEPS_PER_MINUTE);
  // This is the most accurate guess of the pre-homing position we get
  parameters.last_home = homing & HOME_DIR ? parameters.home_pos - steps : parameters.home_pos + steps;
  
  take_steps(backoff, inv_homing_feed);
  // We're now at the home position!
  st.position = parameters.home_pos + backoff;

  // restore hard limits
  delay(10);
  configure_limit_gpio(homing & HOME_DIR, PRESERVE_PULLUP, homing);

  if(homing_end_hook_fun)
    homing_end_hook_fun();
}
