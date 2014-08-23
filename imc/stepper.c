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
#include <mk20dx128.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <pin_config.h>

#include "control_isr.h"
#include "queue.h"
#include "hardware.h"
#include "config.h"
#include "stepper.h"
#include "utils.h"
//||\\!! temp
//#include "../common.h"

static void set_step_events_per_minute(uint32_t); 

#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

enum pulse_status {PULSE_SET, PULSE_RESET};

typedef struct {
  uint32_t pulse_length;
  volatile enum pulse_status step_interrupt_status;
} pulse_state;

static pulse_state pit1_state;

volatile msg_queue_move_t* current_block;

volatile stepper_state_t st;

volatile uint32_t out_step;
volatile uint32_t out_dir;

// hooks
static bool init_hook_set = false;
static void (*init_hook)(void);
static bool step_hook_set = false;
static bool (*step_hook)(void);
static bool exec_hook_set = false;
static bool (*exec_hook)(volatile msg_queue_move_t*);

// Reset all stepper parameters, setup clocks, and make sure
// there is no power to steppers.
void initialize_stepper_state(void){
  // Clock up all of the PITs
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  // Configure PIT 0 - main interrupt timer - as running at minimum steps/minute, but not
  // interrupting.
  set_step_events_per_minute(MINIMUM_STEPS_PER_MINUTE);
  // Configure PIT 1 and 2 - reset timer and sync timer
  PIT_TCTRL1 = TIE;
  PIT_TCTRL2 = TIE;
  // Zero all parameters, and go into idle
  vmemset(&st, 0, sizeof(st));
  current_block = NULL;
  st.state =  STATE_IDLE;
  if(init_hook_set)
    init_hook();
}

// hook registration routines
void set_step_hook(bool (*stephook)(void))
{
  step_hook = stephook;
  step_hook_set = true;
}
void set_execute_hook(bool (*exechook)(volatile msg_queue_move_t *))
{
  exec_hook = exechook;
  exec_hook_set = true;
}
void set_init_hook(void (*inithook)(void))
{
  init_hook = inithook;
  init_hook_set = true;
}

// Enable power to steppers - deassert stepper disable pin
void enable_stepper(void){
  STEPPER_PORT(COR) = DISABLE_BIT;
}
// Power down the stepper motor
void disable_stepper(void){
  STEPPER_PORT(SOR) = DISABLE_BIT;
}


inline static uint32_t iterate_trapezoid_cycle_counter() 
{
  st.trapezoid_tick_cycle_counter += st.cycles_per_step_event;  
  if(st.trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) {
    st.trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
    return(true);
  } else {
    return(false);
  }
}          

// changed this routine to public for control access
uint32_t config_step_timer(uint32_t cycles)
{

  PIT_TCTRL0 &= ~TEN; // Stop the timer 
  PIT_LDVAL0 = cycles; // Load the new value
  PIT_TCTRL0 |= TEN;
  return(cycles);
}

static void set_step_events_per_minute(uint32_t steps_per_minute) 
{
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE){
    steps_per_minute = MINIMUM_STEPS_PER_MINUTE;
  }

  st.cycles_per_step_event = config_step_timer((F_CPU*((uint32_t)60))/steps_per_minute);
}

// new routine to allow module-level access to direction
void set_direction(bool backwards)
{
	out_dir = (uint32_t)backwards;
  // set the direction NOW. Needed for bang-bang control.
  STEPPER_PORT(DOR) = (STEPPER_PORT(DOR) & ~DIR_BIT) | (out_dir ? DIR_BIT : 0);
}
bool get_direction(void)
{
	return out_dir > 0;
}

void enter_sync_state(void)
{
  // Start counting down on timer 2
  //||\\!! temp
  //hid_printf("%u-%i enter_sync_state. Old PIT_LDVAL2 = %u, PIT_CVAL2 = %u\n", get_systick_tenus(), (CONTROL_DDR & SYNC_BIT) ? -1 : CONTROL_PORT(DIR) & SYNC_BIT, PIT_LDVAL2, PIT_CVAL2);
  PIT_LDVAL2 = SYNC_TIMEOUT;
  PIT_TCTRL2 |= TEN;
  // Configure the sync line as high-z input with an interrupt on logic one. I've had issues
  // with triggering on the edge...
  st.state = STATE_SYNC;
  enable_sync_interrupt();
}


void pit0_isr(void) {
  // Set the direction bits. Todo: only do this at the start of a block.
  STEPPER_PORT(DOR) = (STEPPER_PORT(DOR) & ~DIR_BIT) | (out_dir ? DIR_BIT : 0);

  // hook for controller
  if(step_hook_set)
    if(step_hook())
      return;       // step_hook has done everything and asks us to quit here.
  
  if(out_step)
    trigger_pulse();

  if(st.state == STATE_IDLE || st.state == STATE_ERROR){
    // Disable this interrupt
    PIT_TCTRL0 &= ~TEN;
    PIT_TFLG0 = 1;
    return;
  }
  
  if(st.state == STATE_SYNC){ // Done with a block, and done with outputting the last pulse
    enter_sync_state();
    // Allow this to retrigger next time
    PIT_TFLG0 = 1;
    return;
  } 

  out_dir = out_step = 0;
  // Execute step displacement profile by bresenham line algorithm
  if(current_block->length < 0){
    out_dir = 1;
    st.counter -= current_block->length;
  }else{
    st.counter += current_block->length;
  }
  if (st.counter > 0) {
    out_step = 1;
    st.counter -= st.event_count;
  }
  st.step_events_completed++; // Iterate step events

  if(current_block->total_length <= st.step_events_completed){
    // Allow this to trigger once more, to pulse out the last step
    current_block = NULL;
    st.state = STATE_SYNC;
    PIT_TFLG0 = 1;
    return;
  }
  
  // The trapezoid generator always checks step event location to ensure de/ac-celerations are 
  // executed and terminated at exactly the right time. This helps prevent over/under-shooting
  // the target position and speed. 
  // NOTE: By increasing the ACCELERATION_TICKS_PER_SECOND in config.h, the resolution of the 
  // discrete velocity changes increase and accuracy can increase as well to a point. Numerical 
  // round-off errors can effect this, if set too high. This is important to note if a user has 
  // very high acceleration and/or feedrate requirements for their machine.
  if (st.step_events_completed < current_block->stop_accelerating) {
    // Iterate cycle counter and check if speeds need to be increased.
    if ( iterate_trapezoid_cycle_counter() ) {
      st.trapezoid_adjusted_rate += current_block->acceleration;
      if (st.trapezoid_adjusted_rate >= current_block->nominal_rate) {
	// Reached nominal rate a little early. Cruise at nominal rate until decelerate_after.
	st.trapezoid_adjusted_rate = current_block->nominal_rate;
      }
      set_step_events_per_minute(st.trapezoid_adjusted_rate);
    }
  } else if (st.step_events_completed >= current_block->start_decelerating) {
    // Reset trapezoid tick cycle counter to make sure that the deceleration is performed the
    // same every time. Reset to CYCLES_PER_ACCELERATION_TICK/2 to follow the midpoint rule for
    // an accurate approximation of the deceleration curve. For triangle profiles, down count
      // from current cycle counter to ensure exact deceleration curve.
    if (st.step_events_completed == current_block->start_decelerating) {
      if (st.trapezoid_adjusted_rate == current_block->nominal_rate) {
	st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Trapezoid profile
      } else {  
	st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK-st.trapezoid_tick_cycle_counter; // Triangle profile
      }
    } else {
      // Iterate cycle counter and check if speeds need to be reduced.
      if ( iterate_trapezoid_cycle_counter() ) {  
	// NOTE: We will only do a full speed reduction if the result is more than the minimum safe 
	// rate, initialized in trapezoid reset as 1.5 x rate_delta. Otherwise, reduce the speed by
	// half increments until finished. The half increments are guaranteed not to exceed the 
	// CNC acceleration limits, because they will never be greater than rate_delta. This catches
	// small errors that might leave steps hanging after the last trapezoid tick or a very slow
	// step rate at the end of a full stop deceleration in certain situations. The half rate 
	// reductions should only be called once or twice per block and create a nice smooth 
	// end deceleration.
	if (st.trapezoid_adjusted_rate > st.min_safe_rate) {
	  st.trapezoid_adjusted_rate -= current_block->acceleration;
	} else {
	  st.trapezoid_adjusted_rate >>= 1; // Bit shift divide by 2
	}
	if (st.trapezoid_adjusted_rate < current_block->final_rate) {
	  // Reached final rate a little early. Cruise to end of block at final rate.
	    st.trapezoid_adjusted_rate = current_block->final_rate;
	}
	  set_step_events_per_minute(st.trapezoid_adjusted_rate);
      }
    }
  } else {
    // No accelerations. Make sure we cruise exactly at the nominal rate.
    if (st.trapezoid_adjusted_rate != current_block->nominal_rate) {
      st.trapezoid_adjusted_rate = current_block->nominal_rate;
      set_step_events_per_minute(st.trapezoid_adjusted_rate);
    }
  }            
  PIT_TFLG0 = 1;
} 



inline void trigger_pulse(void){
#ifdef STEP_PULSE_DELAY
  pit1_state.step_interrupt_status = PULSE_SET;
  PIT_LDVAL1 = STEP_PULSE_DELAY;
#else
  STEPPER_PORT(TOR) = STEP_BIT;
  PIT_LDVAL1 = pit1_state.pulse_length;
#endif
  PIT_TCTRL1 |= TEN;
  if (out_dir) { st.position--; }
    else { st.position++; }
}

void pit1_isr(void){
  PIT_TFLG1 = 1;
  PIT_TCTRL1 &= ~TEN;
  STEPPER_PORT(COR) = STEP_BIT;   // was TOR; changed for reliability
#ifdef STEP_PULSE_DELAY
  if(pit1_state.step_interrupt_status == PULSE_SET){
    pit1_state.step_interrupt_status = PULSE_RESET;
    PIT_LDVAL1 = pit1_state.pulse_length;
    PIT_TCTRL1 |= TEN;  
  }
  #endif
}

void execute_move(void){
  // Attempt to pop a block off the queue
  current_block = dequeue_block();
  if(current_block == NULL){
    // If this doesn't work, we're out of moves and go into idle.
    st.state = STATE_IDLE;
    float_sync_line();
    return;
  }
  enable_stepper();
  st.state = STATE_EXECUTE;
  // Trigger the ISR to pull down the sync line after a given propogation dela
  trigger_sync_delay();

  // trigger the hook. If it returns true, skip the rest of the init (it will be handled elsewhere)
  if(exec_hook_set)
    if(exec_hook(current_block))
      return;

  // Initialize this trapezoid generator for this block
  st.trapezoid_adjusted_rate = current_block->initial_rate;
  set_step_events_per_minute(st.trapezoid_adjusted_rate); // Initialize cycles_per_step_event
  st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Start halfway for midpoint rule.
  st.min_safe_rate = (3 * current_block->acceleration) >> 1; // 1.5 x rate_delta
  st.counter = -(current_block->total_length >> 1);
  st.event_count = current_block->total_length;
  st.step_events_completed = 0;     
  out_dir = out_step = 0;
  // Then immediately trigger the step more or less immediately
  PIT_TCTRL0 &= ~TEN;
  PIT_LDVAL0 = 100;
  PIT_TCTRL0 |= TEN | TIE;
}

// Immediately kill all motion, probably killing position due to deceleration
void stop_motion(void){
  PIT_TCTRL0 &= ~TEN;
}

// name changed from get_position() to avoid confusion with encoder position
int32_t get_motor_position(void){
  return st.position;
}
void set_motor_position(uint32_t p){
  st.position = p;
}
