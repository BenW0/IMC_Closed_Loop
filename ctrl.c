/********************************************************************************
 * Controller Module
 * Ben Weiss, University of Washington 2014
 * Purpose: Implements the control loop for the system.
 * 
 * Source: 
 * Some of the PIT code is modified from Matthew Sorensen's code in stepper.c
 * 
 * License: None. This is for internal development only!
 ********************************************************************************/

#include "common.h"
#include <usb_serial.h>
#include <string.h>
#include <stdio.h>

#include "ctrl.h"
#include "qdenc.h"
#include "spienc.h"
#include "path.h"
#include "stepper.h"
#include <pin_config.h>

// Type Definitions ==================================================================
typedef struct
{
  int32_t position;
  float velocity;
  float target_pos;
  float target_vel;
  int32_t motor_position;
} hist_data_t;

// Constants =========================================================================
#define HIST_SIZE   2048     // have the history use ~40k of memory. Needs to be a power of 2.

// Global Variables ==================================================================
extern uint32_t enc_tics_per_step;
extern float steps_per_enc_tic;
float pid_kp = 0.f, pid_ki = 0.f, pid_kd = 0.f;

// Local Variables ===================================================================
static ctrl_mode mode = CTRL_DISABLED;
static uint32_t ctrl_period_cycles;		// set update time, in cpu cycles
static float ctrl_period_sec;         // set update time, in seconds
static volatile uint32_t update_time = 0;		// set to the time the last update took, in cpu cycles
static volatile hist_data_t hist_data[HIST_SIZE];
static volatile uint32_t hist_head = 0;


// PID variables
static volatile float pid_i_sum = 0.f, pid_last_err = 0.f;

// Function Predeclares ==============================================================
void set_update_cycles(uint32_t cycles);
float pid_ctrl(float dt, float target_pos, float target_vel, int32_t encpos);

// Initializes the PIT timer used for control
void init_ctrl(void)
{
	// Clock up all of the PITs
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  PIT_TCTRL3 = PIT_TCTRL_TIE_MASK;
	
  NVIC_ENABLE_IRQ(IRQ_PIT_CH3);
	//ctrl_set_period(5000);		// start with 5ms update frequency
	
	// Controller heartbeat (for checking clock regularity)
	GPIOD_PDDR |= (1<<3);
	PORTD_PCR3 = STANDARD_OUTPUT;

  // clear the history ringbuffer
  memset((void *)hist_data, 0, sizeof(hist_data_t) * HIST_SIZE);
}

// Sets the frequency of the controller update
// pass update period in us.
void ctrl_set_period(uint32_t us)
{
	ctrl_period_cycles = ((F_BUS / 1000L) * us) / 1000L;
  ctrl_period_sec = (float)us / 1000000.f;
	set_update_cycles(ctrl_period_cycles);
}
uint32_t ctrl_get_period(void)
{
  return ((ctrl_period_cycles * 1000L) / F_BUS) * 1000L;
}

// sets the timer value for the PIT in bus clock cycles (same as cpu cycles for 48 MHz chip)
void set_update_cycles(uint32_t cycles)
{
	PIT_TCTRL3 &= ~PIT_TCTRL_TEN_MASK; // Stop the timer 
  PIT_LDVAL3 = cycles; // Load the new value (division is split to save roundoff and not overflow)
  PIT_TCTRL3 |= PIT_TCTRL_TEN_MASK;
}

// Enables/disables the controller. mode = CTRL_DISABLE turns off the controller.
// otherwise specifies which control algorithm to use.
void ctrl_enable(ctrl_mode mode)
{
  if(mode != CTRL_DISABLED)
  {
    // need to clear PID variables to avoid major issues
    pid_i_sum = 0;
    pid_last_err = 0;
    set_update_cycles(ctrl_period_cycles);
  }
  else    // disable control
    PIT_TCTRL3 &= ~PIT_TCTRL_TEN_MASK;
}


// gets the most recent control law update time (time to perform the calculations for the controller)
// returns ms.
float ctrl_get_update_time(void)
{
	return (float)update_time / (float)F_BUS * 1000;
}

// spits the history ringbuffer out over USB.
void output_history(void)
{
  uint32_t old_head_loc;
  // start at the tail and write to the end of the buffer, then catch back up to the head
  // (which may move...)
  old_head_loc = hist_head;
  usb_serial_write((void*)(hist_data + old_head_loc + 1), sizeof(hist_data_t) * (HIST_SIZE - old_head_loc - 1));
  usb_serial_write((void*)(hist_data), sizeof(hist_data_t) * old_head_loc);
}


// Controller ISR - fires every step_freq us 
void pit3_isr(void) 
{
	uint32_t old_systic, new_systic;
	int32_t encpos;
  float target_pos, target_vel, newvel = enc_tics_per_step;
	
	// get the current system tick timer so we can add the time it takes to do the loop update into the update frequency
	// remember, the timer counts down.
	old_systic = SYST_CVR;
	
	// Read the encoder position
	//||\\!! TODO: figure out what happens if the encoder has lost track...
  get_enc_value(&encpos);

  // get the path target location
  path_get_target(&target_pos, &target_vel);
  // convert into encoder tics/minute instead of motor steps/minute
  target_pos = target_pos * enc_tics_per_step;
  target_vel = target_vel * enc_tics_per_step;
	
	// perform the control law
  switch(mode)
  {
  case CTRL_PID :
    newvel = pid_ctrl(ctrl_period_sec, target_pos, target_vel, encpos);
    break;
  case CTRL_DISABLED :
    // disable this interrupt
    PIT_TCTRL3 &= ~PIT_TCTRL_TEN_MASK;
    // clear the interrupt flag
    PIT_TFLG3 = 1;
    return;
  }

  // save this update to the ring buffer
  hist_head = (hist_head + 1) & (HIST_SIZE - 1);   // list_size is a power of 2, so list_size - 1 is 0b0..01..1
  hist_data[hist_head].motor_position = get_motor_position();
  hist_data[hist_head].position = encpos;
  hist_data[hist_head].target_pos = target_pos;
  hist_data[hist_head].target_vel = target_vel;
  hist_data[hist_head].velocity = newvel;

  // the output was encoder tics per minute; we want that back in motor steps/minute
  newvel = newvel * steps_per_enc_tic;
  // set the new velocity
  set_step_events_per_minute((uint32_t)newvel);
	
	// Update the controller heartbeat
	GPIOD_PTOR = (1<<3);
	
	// reset the PIT timer's cycle time based on how long this took.
	new_systic = SYST_CVR;
	if(new_systic < old_systic)		// counter counts down!
		update_time = old_systic - new_systic;
	else	// counter rolled over
		update_time = old_systic + SYST_RVR - new_systic;
	if(update_time > ctrl_period_cycles)
		set_update_cycles(ctrl_period_cycles - update_time);
	else
		set_update_cycles(100);		// wait some minimum time before firing again
	
  // clear the interrupt flag
  PIT_TFLG3 = 1;
}


// PID controller
// Computes the PID control signal given information about the current system state.
// params:
//   dt - loop update period (sec)
//   target_pos - target position (in encoder units)
//   target_vel - target velocity (in encoder units/sec)
//   encpos - current (actual) position
// Returns the control input for the system (update speed in step events per minute)
// uses module variables beginning in pid_ only.
float pid_ctrl(float dt, float target_pos, float target_vel, int32_t encpos)
{
  float err = target_pos - encpos, ctrl;
  // update the integrator
  pid_i_sum += err * dt;

  // control law
  ctrl = pid_kp * err + pid_ki * pid_i_sum + pid_kd * (err - pid_last_err) / dt;

  // update history
  pid_last_err = err;

  return ctrl;
}



