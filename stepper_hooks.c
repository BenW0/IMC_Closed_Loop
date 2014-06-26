#include "common.h"
#include <string.h>
#include <pin_config.h>

#include "imc/hardware.h"
#include "imc/stepper.h"
#include "imc/parameters.h"
#include "imc/homing.h"

#include "ctrl.h"
#include "path.h"
#include "qdenc.h"
#include "spienc.h"


// Global Variables ==================================================================
bool old_stepper_mode = false;      // use stock IMC stepper isr code.
extern float enc_tics_per_step;
extern float steps_per_enc_tic;

// Local Variables ===================================================================
volatile int32_t steps_to_go = -1;
static uint32_t old_steps_per_minute = MINIMUM_STEPS_PER_MINUTE;
static uint32_t new_cycles_per_step_event = MINIMUM_STEPS_PER_MINUTE;
bool force_steps_per_minute = true;   // force the stepper module to reset its counter ever update?
static ctrl_mode old_ctrl_mode;       // used in homing to store the old control mode while control is disabled.
//static char message[100];

// Function Predeclares ==============================================================
bool step_hook();
bool exec_hook(volatile msg_queue_move_t *);

bool homing_start_hook();
void homing_end_hook();


void init_stepper_hooks(void){
  set_step_hook(step_hook);
  set_execute_hook(exec_hook);
  set_homing_hooks(homing_start_hook, homing_end_hook);
}


void set_step_events_per_minute_ctrl(uint32_t steps_per_minute) 
{
  if (steps_per_minute < 1){//MINIMUM_STEPS_PER_MINUTE){
    steps_per_minute = 1;//MINIMUM_STEPS_PER_MINUTE;
  }
  // we want to avoid two scenarios:
  // 1) waiting too long because we reset the timer just before it triggered
  // 2) waiting too long because the new steps_per_minute is high but the current one is very low and we didn't reset.
  // so, if the new timer value is less than the current remaining time, we'll restart the timer.
  
  // set this as the new value to be implemented on the next interrupt
  new_cycles_per_step_event = (F_CPU*((uint32_t)60))/steps_per_minute;
  
  if (force_steps_per_minute || new_cycles_per_step_event < PIT_CVAL0)
  {
    // reset the timer to count down from the new value.
    st.cycles_per_step_event = config_step_timer(new_cycles_per_step_event);
  }
}

uint32_t get_step_events_per_minute(void)
{
  return (F_CPU*((uint32_t)60))/new_cycles_per_step_event;
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

// step_hook - hook into the stepper ISR, executed just after
// the direction bit is set, but before any steps are fired.
// Returns true if the stepping is complete (i.e. the controller
// is active and we don't want legacy behavior) and false if
// the imc controller should continue with its normal behavior.
bool step_hook(void) {
  // should we keep legacy open-loop behavior?
  if(old_stepper_mode)
    return false;     // have the isr continue with stock IMC code
  

  // check for end stop states (this needed as a safety net for testing) //||\\!
  if(get_direction())   // going backwards?
  {
    // logic: Trigger a pulse if we are going forward, and
    // if the endstop is not enabled, step anyway.
    // if the endstop pin matches the invert flag, we can step.
    if(parameters.homing & ENABLE_MIN && (((CONTROL_PORT(DIR) & MIN_LIMIT_BIT) ? 1 : 0) != (parameters.homing & INVERT_MIN ? 1 : 0)))
    {
      // the endstop is asserted.
      sprintf(message, "'Min Endstop Assert!\n");
      usb_serial_write(message, strlen(message));
    }
    else
      trigger_pulse();
  }
  else    // going forwards
  {
    if(parameters.homing & ENABLE_MAX && (((CONTROL_PORT(DIR) & MAX_LIMIT_BIT) ? 1 : 0) != (parameters.homing & INVERT_MAX ? 1 : 0)))
    {
      // the endstop is asserted.
      sprintf(message, "'Max Endstop Assert!\n");
      usb_serial_write(message, strlen(message));
    }
    else
      trigger_pulse();
  }


  // are we out of steps on a steps-limited move?
  if(steps_to_go >= 0)
  {
    if(--steps_to_go == 0)
    {
      set_step_events_per_minute_ctrl(old_steps_per_minute); // restore old step rate
      st.state = STATE_IDLE;  // disable the timer
      steps_to_go = -1; // disable the step counter
      PIT_TCTRL0 &= ~TEN;
      PIT_TFLG0 = 1;
      return true;
    }
    // are we close? (if so, slow down to safe speed)
    if(steps_to_go < 30)
      set_step_events_per_minute_ctrl(max(get_step_events_per_minute() - 400, MINIMUM_STEPS_PER_MINUTE));
  }
  
  if(st.state == STATE_IDLE || st.state == STATE_ERROR){
    // Disable this interrupt
    PIT_TCTRL0 &= ~TEN;
    PIT_TFLG0 = 1;
    return true;
  }
  // clear the interrupt flag
  PIT_TFLG0 = 1;
  
  // load new cycle count
  st.cycles_per_step_event = config_step_timer(new_cycles_per_step_event);
  return true;
} 


void start_moving(void){

  st.state = STATE_EXECUTE;
  old_steps_per_minute = get_step_events_per_minute();

  //PIT_TCTRL0 &= ~TEN;
  //PIT_LDVAL0 = 48;
  PIT_TCTRL0 |= TEN | TIE;
}


// exec_hook: Hook called when executing a move.
// I'm not sure this really belongs here (maybe should be in path?)
// but it's implemented as a hook in imc/stepper.c, so here it is.
// As with step_hook, returns true if we've handled the event
// and false if legacy behavior should be implemented.
bool exec_hook(volatile msg_queue_move_t *current_block)
{
  if(old_stepper_mode)
    return false;   // don't do anything with path.c; retain legacy behavior.
  
  // tell the path module to run a block.
  if(current_block)
    path_ramps_move(current_block);
  return true;
}

// homing_start_hook: Hook called when the homing routine is begun.
// We will use this to disable the controller while homing. Otherwise the
// homing movement could be seen as a disturbance and the controller might try
// to correct for it.
//
// NOTE: If we returned true, we would override the default imc homing routine.
bool homing_start_hook()
{
  old_ctrl_mode = ctrl_get_mode();
  ctrl_enable(CTRL_DISABLED);
  serial_printf("'Homing Start Hook\n");
  return false;
}
void homing_end_hook()
{
  // set the encoder position.
  set_enc_value((int32_t)((float)get_motor_position() * enc_tics_per_step));
  serial_printf("'Homing End Hook\n");
  // re-enable control.
  ctrl_enable(old_ctrl_mode);
}