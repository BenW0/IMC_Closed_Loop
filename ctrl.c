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
#include <math.h>

#include "ctrl.h"
#include "qdenc.h"
#include "spienc.h"
#include "path.h"
#include "stepper.h"
#include <pin_config.h>

// Type Definitions ==================================================================
typedef struct
{
  uint32_t time;
  int32_t position;
  float velocity;
  float cmd_velocity;
  float target_pos;
  float target_vel;
  int32_t motor_position;
} hist_data_t;

// Constants =========================================================================
#define HIST_SIZE   2048     // have the history use ~40k of memory. Needs to be a power of 2.
#define FF_TARGETS 16        // Feed forward target buffer size. another ring buffer...needs to be a power of 2.

// Global Variables ==================================================================
extern float enc_tics_per_step;
extern float steps_per_enc_tic;
float pid_kp = 0.f, pid_ki = 0.f, pid_kd = 0.f;
float min_ctrl_vel = 0;
float max_ctrl_vel = 1.0e6;      //maximum velocity my test motor can support consistently without stalling.
bool pos_ctrl_mode = false;       // controllers output new position target which gets converted to velocity.
uint32_t ctrl_feedforward_advance = 0;

// Local Variables ===================================================================
static ctrl_mode mode = CTRL_DISABLED;
static uint32_t ctrl_period_cycles;		// set update time, in cpu cycles
static float ctrl_period_sec;         // set update time, in seconds
static volatile uint32_t update_time = 0;		// set to the time the last update took, in cpu cycles
static volatile hist_data_t hist_data[HIST_SIZE];
static volatile uint32_t hist_head = 0;
static uint32_t hist_time_offset = 0;
static char message[100] = "Hello, World";
static volatile float last_vel = 0;   // velocity chosen last update
static volatile int32_t last_encpos = 0.f;
static volatile real ctrl_integrator = 0.f;   // control integrator variable.
static volatile real ff_target_pos_buf[FF_TARGETS];
static volatile real ff_target_vel_buf[FF_TARGETS];
static volatile uint32_t ff_target_head = 0;


// PID variables
static volatile float pid_i_sum = 0.f;

// Function Predeclares ==============================================================
void set_update_cycles(uint32_t cycles);
real pid_ctrl(real dt, real target_pos, real target_vel, real encpos, real lastvel);
void bang_ctrl(real dt, real target_pos, real target_vel, real encpos);

// Initializes the PIT timer used for control
void init_ctrl(void)
{
	// Clock up all of the PITs
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  PIT_TCTRL3 = PIT_TCTRL_TIE_MASK;
	
  NVIC_ENABLE_IRQ(IRQ_PIT_CH3);
	ctrl_set_period(5000);		// start with 5ms update frequency
  ctrl_enable(CTRL_DISABLED); // disable the timer which ctrl_set_period just enabled
	
	// Controller heartbeat (for checking clock regularity)
	GPIOD_PDDR |= (1<<3);
	PORTD_PCR3 = STANDARD_OUTPUT;

  // clear the history ringbuffer
  memset((void *)hist_data, 0, sizeof(hist_data_t) * HIST_SIZE);
  memset((void *)ff_target_pos_buf, 0, sizeof(real) * FF_TARGETS);
  memset((void *)ff_target_vel_buf, 0, sizeof(real) * FF_TARGETS);
}

// Sets the frequency of the controller update
// pass update period in us.
void ctrl_set_period(uint32_t us)
{
	ctrl_period_cycles = (F_BUS / 1000000L) * us;
  ctrl_period_sec = (float)us / 1000000.f;
	set_update_cycles(ctrl_period_cycles);
}
uint32_t ctrl_get_period(void)
{
  return (ctrl_period_cycles) / (F_BUS / 1000000L);
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
void ctrl_enable(ctrl_mode newmode)
{
  if(newmode != CTRL_DISABLED)
  {
    // need to clear PID variables to avoid major issues
    pid_i_sum = 0;
    get_enc_value(&last_encpos);
    last_vel = 0;
    ctrl_integrator = 0;
    ff_target_head = 0;
    memset((void *)ff_target_pos_buf, 0, sizeof(real) * FF_TARGETS);
    memset((void *)ff_target_vel_buf, 0, sizeof(real) * FF_TARGETS);
    // if the mode has changed, reset the history buffer
    if(newmode != mode)
    {
      memset((void *)hist_data, 0, sizeof(hist_data_t) * HIST_SIZE);
      hist_head = 0;
      hist_time_offset = get_systick_tenus();   // so we don't have some 0's and then stuff way off in time at the same time
    }

    set_update_cycles(ctrl_period_cycles);

    if(CTRL_BANG == newmode)
    {
      // turn "off" the motor update period
      set_step_events_per_minute(1);
    }
  }
  else    // disable control
    PIT_TCTRL3 &= ~PIT_TCTRL_TEN_MASK;
  mode = newmode;
}


// gets the most recent control law update time (time to perform the calculations for the controller)
// returns ms.
float ctrl_get_update_time(void)
{
	return (float)update_time * 1000.f / (float)F_BUS;
}

// spits the history ringbuffer out over USB.
void output_history(void)
{
  uint32_t old_head_loc;
  // start at the tail and write to the end of the buffer, then catch back up to the head
  // (which may move...)
  old_head_loc = hist_head;
  sprintf(message, "%lu\n", HIST_SIZE);   // # of entries we're going to print
  usb_serial_write(message, strlen(message));
  // the serial port can't take all this data at once, so we'll give it to them in bites...
  for(uint32_t chunk = old_head_loc + 1; chunk < HIST_SIZE - 1; chunk += 100)
  {
    usb_serial_write((void*)(hist_data + chunk), sizeof(hist_data_t) * min(HIST_SIZE - chunk, 100));
    delay_real(30);
  }
  for(uint32_t chunk = 0; chunk < old_head_loc + 1; chunk += 100)
  {
    usb_serial_write((void*)(hist_data + chunk), sizeof(hist_data_t) * min(old_head_loc + 1 - chunk, 100));
    delay_real(30);
  }
}


// Controller ISR - fires every ctrl_period_cycles cycles = ctrl_period_sec seconds
// The timing on this routine will break down if the control algorithm takes more than SYSTICK_UPDATE_MS ms to
// to it's job.
void pit3_isr(void) 
{
	uint32_t old_systic, new_systic, time_of_update;
	int32_t encpos, motorpos;
  real target_pos, target_vel, ctrl_out;
	
	// Update the controller heartbeat
	GPIOD_PTOR = (1<<3);

	// get the current system tick timer so we can add the time it takes to do the loop update into the update frequency
	// remember, the timer counts down.
	old_systic = SYST_CVR;
  time_of_update = get_systick_tenus();   // do this just once so we don't change our control if an unknown time elapses between querying position and doing control things
	
	// Read the encoder position
	//||\\!! TODO: figure out what happens if the encoder has lost track...
  get_enc_value(&encpos);
  motorpos = get_motor_position();
  last_vel = (encpos - last_encpos) / ctrl_period_sec * 60;   // tics/min
  

  target_pos = ff_target_pos_buf[(ff_target_head - ctrl_feedforward_advance) & (FF_TARGETS - 1)];
  target_vel = ff_target_vel_buf[(ff_target_head - ctrl_feedforward_advance) & (FF_TARGETS - 1)];
	
	// perform the control law
  switch(mode)
  {
  case CTRL_UNITY :
    if(pos_ctrl_mode)
      ctrl_out = 0;   // no control signal
    else
      ctrl_out = target_vel;
    break;
  case CTRL_PID :
    ctrl_out = pid_ctrl(ctrl_period_sec, target_pos, target_vel, encpos, last_vel);
    break;
  case CTRL_BANG :
    bang_ctrl(ctrl_period_sec, target_pos, target_vel, encpos);
    ctrl_out = 0;   // bang-bang doesn't use velocity.
    break;
  default :
    // disable this interrupt
    PIT_TCTRL3 &= ~PIT_TCTRL_TEN_MASK;
    // clear the interrupt flag
    PIT_TFLG3 = 1;
    return;
  }

  if(pos_ctrl_mode)
  {
    // in this mode, "ctrl_out", the output of the controller, gets interpreted as a new position error, 
    // and actual velocity is computed from the error between the position target and the current MOTOR position.
    // (calculation is still done in tics so we maintain compatibility with the normal mode)
    // see notes on 5/8/14
    // integrate ctrl_out:
    //ctrl_integrator += ctrl_out * ctrl_period_sec;
    //ctrl_out = ctrl_integrator;
    ctrl_out += ff_target_pos_buf[ff_target_head];   // use the advanced feedforward target here to null out the system lag.
    ctrl_out = -((real)motorpos * enc_tics_per_step - ctrl_out) / ctrl_period_sec * 60;    // see notebook, 5/7/14
  }

  // clamp the new velocity
  if(fabsf(ctrl_out) < min_ctrl_vel) ctrl_out = 0;
  if(fabsf(ctrl_out) > max_ctrl_vel) ctrl_out = copysignf(max_ctrl_vel, ctrl_out);
  

  // save this update to the ring buffer
  hist_head = (hist_head + 1) & (HIST_SIZE - 1);   // list_size is a power of 2, so list_size - 1 is 0b0..01..1
  hist_data[hist_head].time = time_of_update - hist_time_offset;  // rollover may occur here, but this is just reporting.
  hist_data[hist_head].motor_position = motorpos * enc_tics_per_step;
  hist_data[hist_head].position = encpos;
  hist_data[hist_head].target_pos = target_pos;
  hist_data[hist_head].target_vel = target_vel;
  hist_data[hist_head].velocity = last_vel;
  hist_data[hist_head].cmd_velocity = ctrl_out;


  // the output was encoder tics per minute; we want that back in motor steps/minute
  ctrl_out = ctrl_out * steps_per_enc_tic;
  // set the new velocity
  if(mode != CTRL_BANG)
  {
    // don't do this when we're in bang mode...it does it internally.
    set_direction(ctrl_out > 0.f ? true : false);
    set_step_events_per_minute((uint32_t)abs((int32_t)floorf(ctrl_out)));
  }
  
  // get the path target location (encoder tics) and velocity (encoder tics/minute) for the NEXT update (even including feedforward).
  // We will get the target advanced in time ctrl_feedforward_advance steps + 1 and keep it until it's current.
  ff_target_head = (ff_target_head + 1) & (FF_TARGETS - 1);   // advance the head
  path_get_target(ff_target_pos_buf + ff_target_head, ff_target_vel_buf + ff_target_head, time_of_update + (ctrl_feedforward_advance + 1) * ctrl_period_sec * 100000.f);

	
  
  last_encpos = encpos;
	
 // __disable_irq();    // keep us from being interrupted for a sec
	new_systic = SYST_CVR;

	if(new_systic < old_systic)		// counter counts down!
		update_time = old_systic - new_systic;
	else	// counter rolled over
		update_time = old_systic - new_systic + SYST_RVR;
  
	// reset the PIT timer's cycle time based on how long this took. That's not how PIT timers
  // work; the following code is irrelevant (but taught me something!)
	//if(update_time < ctrl_period_cycles)
 // {
	//	set_update_cycles(ctrl_period_cycles - update_time);
 //   //sprintf(message, "Old: %lu; New: %lu; Diff: %li; Target: %lu, Set: %lu\n", old_systic, new_systic, (long int)old_systic - (long int)new_systic, ctrl_period_cycles, ctrl_period_cycles - update_time);
 // }
	//else
 // {
	//	set_update_cycles(100);		// wait some minimum time before firing again
 //   //sprintf(message, "Old: %lu; New: %lu; Diff: %li; Target: %lu, Set: %lu\n", old_systic, new_systic, (long int)old_systic - (long int)new_systic, ctrl_period_cycles, 100);
 // }
 // 
 // //usb_serial_write(message, strlen(message));
	//
  // clear the interrupt flag
  PIT_TFLG3 = 1;
//  __enable_irq();     // done with time critical section
}


// PID controller
// Computes the PID control signal given information about the current system state.
// params:
//   dt - loop update period (sec)
//   target_pos - target position (in encoder units)
//   target_vel - target velocity (in encoder units/sec)
//   encpos - current (actual) position
//   lastvel - current velocity (that chosen by last update)
// Returns the control input for the system (update speed in step events per minute)
// uses module variables beginning in pid_ only.
real pid_ctrl(real dt, real target_pos, real target_vel, real encpos, real lastvel)
{
  real err = target_pos - encpos, ctrl;
  // update the integrator
  pid_i_sum += err * dt;

  // control law
  ctrl = pid_kp * err + pid_ki * pid_i_sum + pid_kd * (target_vel - lastvel);

  return ctrl;
}


// Bang-Bang controller
// Answers the question "should we take a step?" based solely on position error.
// This works because our cost of switching directions is almost free.
void bang_ctrl(real dt, real target_pos, real target_vel, real encpos)
{
  if(fabsf(encpos - target_pos) > enc_tics_per_step)
  {
    if(encpos < target_pos)
    {
      set_direction(true);
      //sprintf(message, "enc: %f target: %f diff: %f FORWARD\n", encpos, target_pos, encpos - target_pos);   // # of entries we're going to print
      //usb_serial_write(message, strlen(message));
    }
    else
    {
      set_direction(false);
      //sprintf(message, "enc: %f target: %f diff: %f BACKWARDS\n", encpos, target_pos, encpos - target_pos);   // # of entries we're going to print
      //usb_serial_write(message, strlen(message));
    }
    trigger_pulse();    // backdoor to fire a step RIGHT NOW
  }
}



