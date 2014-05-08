#include "common.h"
#include <string.h>
#include <pin_config.h>

#include "hardware.h"
#include "stepper.h"



#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

enum pulse_status {PULSE_SET, PULSE_RESET};

typedef struct {
  uint32_t pulse_length;
  volatile enum pulse_status step_interrupt_status;
} pulse_state;

static pulse_state pit1_state;

volatile stepper_state_t st;

volatile uint32_t out_step;
volatile uint32_t out_dir;

volatile int32_t steps_to_go = -1;

static uint32_t old_steps_per_minute = MINIMUM_STEPS_PER_MINUTE;

// Reset all stepper parameters, setup clocks, and make sure
// there is no power to steppers.
void initialize_stepper_state(void){
  // Clock up all of the PITs
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  // Configure PIT 0 - main interrupt timer - as running at minimum steps/minute, but not
  // interrupting.
  set_step_events_per_minute(MINIMUM_STEPS_PER_MINUTE);
  // Configure PIT 0, 1 and 2 - step timer, reset timer and sync timer
	PIT_TCTRL0 = TIE;
  PIT_TCTRL1 = TIE;
  PIT_TCTRL2 = TIE;
  // Start in the idle state, but first wake up to check for keep steppers enabled option.
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH2);
  // Zero all parameters, and start at a very low step rate.
  memset((void*)&st, 0, sizeof(st));
  //current_block = NULL;
  //st.state =  STATE_IDLE;
	set_step_events_per_minute(1);
	st.state = STATE_EXECUTE;
}
// Enable power to steppers - deassert stepper disable pin
void enable_stepper(void){
  STEPPER_PORT(COR) = DISABLE_BIT;
}
// Power down the stepper motor
void disable_stepper(void){
  STEPPER_PORT(SOR) = DISABLE_BIT;
}


uint32_t config_step_timer(uint32_t cycles)
{

  PIT_TCTRL0 &= ~TEN; // Stop the timer 
  PIT_LDVAL0 = cycles; // Load the new value
  PIT_TCTRL0 |= TEN;
  return(cycles);
}

void set_step_events_per_minute(uint32_t steps_per_minute) 
{
  if (steps_per_minute < 1){//MINIMUM_STEPS_PER_MINUTE){
    steps_per_minute = 1;//MINIMUM_STEPS_PER_MINUTE;
  }
  // we want to avoid two scenarios:
  // 1) waiting too long because we reset the timer just before it triggered
  // 2) waiting too long because the new steps_per_minute is high but the current one is very low and we didn't reset.
  // so, if the new timer value is less than the current remaining time, we'll restart the timer.
  
  // set this as the new value to be implemented on the next interrupt
  st.new_cycles_per_step_event = (F_CPU*((uint32_t)60))/steps_per_minute;
  
  if (st.new_cycles_per_step_event < PIT_CVAL0)
  {
    // reset the timer to count down from the new value.
    st.cycles_per_step_event = config_step_timer(st.new_cycles_per_step_event);
  }
}

uint32_t get_step_events_per_minute(void)
{
  return (F_CPU*((uint32_t)60))/st.new_cycles_per_step_event;
}

void set_direction(bool forward)
{
	out_dir = (uint32_t)forward;
  // set the direction NOW. Needed for bang-bang control.
  STEPPER_PORT(DOR) = (STEPPER_PORT(DOR) & ~DIR_BIT) | (out_dir ? DIR_BIT : 0);
}

bool get_direction(void)
{
	return out_dir > 0;
}

// Limit the move to a set number of steps.
void set_steps_to_go(int32_t steps)
{
  if(steps < 0)
    steps = -1;
  steps_to_go = steps;
}

int32_t get_steps_to_go(void)
{
  return steps_to_go;
}

void pit0_isr(void) {
  // Set the direction bits. Todo: only do this at the start of a block.
  STEPPER_PORT(DOR) = (STEPPER_PORT(DOR) & ~DIR_BIT) | (out_dir ? DIR_BIT : 0);
  // fire a step
  trigger_pulse();


  // are we out of steps on a steps-limited move?
  if(steps_to_go >= 0)
  {
    if(--steps_to_go == 0)
    {
      set_step_events_per_minute(old_steps_per_minute); // restore old step rate
      st.state = STATE_IDLE;  // disable the timer
      steps_to_go = -1; // disable the step counter
      PIT_TCTRL0 &= ~TEN;
      PIT_TFLG0 = 1;
      return;
    }
    // are we close? (if so, slow down to safe speed)
    if(steps_to_go < 30)
      set_step_events_per_minute(max(get_step_events_per_minute() - 400, MINIMUM_STEPS_PER_MINUTE));
  }
  
  if(st.state == STATE_IDLE || st.state == STATE_ERROR){
    // Disable this interrupt
    PIT_TCTRL0 &= ~TEN;
    PIT_TFLG0 = 1;
    return;
  }
  // clear the interrupt flag
  PIT_TFLG0 = 1;
  
  // load new cycle count
  st.cycles_per_step_event = config_step_timer(st.new_cycles_per_step_event);
} 



inline void trigger_pulse(void){
#ifdef STEP_PULSE_DELAY
  pit1_state.step_interrupt_status = PULSE_SET;
  PIT_LDVAL1 = STEP_PULSE_DELAY;
#else
  STEPPER_PORT(SOR) = STEP_BIT;   //||\\!! was TOR
  PIT_LDVAL1 = pit1_state.pulse_length;
#endif
  PIT_TCTRL1 |= TEN;
	st.position += out_dir ? 1 : -1;
}

void pit1_isr(void){
  PIT_TFLG1 = 1;
  PIT_TCTRL1 &= ~TEN;
  STEPPER_PORT(COR) = STEP_BIT;   //||\\!! Was TOR
#ifdef STEP_PULSE_DELAY
  if(pit1_state.step_interrupt_status == PULSE_SET){
    pit1.step_interrupt_status = PULSE_RESET;
    PIT_LDVAL1 = pit1_state.pulse_length;
    PIT_TCTRL1 |= TEN;  
  }
  #endif
}

void execute_move(void){

  st.state = STATE_EXECUTE;
  old_steps_per_minute = get_step_events_per_minute();

  //PIT_TCTRL0 &= ~TEN;
  //PIT_LDVAL0 = 48;
  PIT_TCTRL0 |= TEN | TIE;
}

// Immediately kill all motion, probably killing position due to deceleration
void stop_motion(void){
  //disable_stepper();
  PIT_TCTRL0 &= ~TEN;
}

int32_t get_motor_position(void){
  return st.position;
}
void set_motor_position(uint32_t p){
  st.position = p;
}
