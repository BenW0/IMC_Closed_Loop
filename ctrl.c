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
#include "stepper_hooks.h"
#include <pin_config.h>
#include "imc/utils.h"
#include "imc/stepper.h"

// Type Definitions ==================================================================
typedef struct
{
  uint32_t time;
  int32_t position;
  //float velocity;
  float pos_error_deriv;
  float cmd_velocity;
  float target_pos;
  float target_vel;
  int32_t motor_position;
} hist_data_t;

// Constants =========================================================================
#define HIST_SIZE   1600U     // have the history use ~40k of memory.
#define FF_TARGETS 16        // Feed forward target buffer size. another ring buffer...needs to be a power of 2.

// Global Variables ==================================================================
extern float enc_tics_per_step;
extern float steps_per_enc_tic;
extern bool old_stepper_mode;
float pid_kp = 0.f, pid_ki = 0.f, pid_kd = 0.f;
float min_ctrl_vel = 0;
float max_ctrl_vel = 21.0e6;      //maximum velocity my test motor can support consistently without stalling.
bool pos_ctrl_mode = true;       // controllers output new position target which gets converted to velocity.
uint32_t ctrl_feedforward_advance = 0;
float fault_thresh = 10.f;        // change in error between commanded and actual position that triggers a fault condition.
real osac_As[10] = {0., 0.};      // A is assumed monic, so all we store is A1..A10
real osac_Bs[10] = {1.};          // B is not monic, so we store B0..B9
uint32_t osac_Acount = 2, osac_Bcount = 1;
bool stream_ctrl_hist = false;    // turn on streaming of control history over usb in real time.

real darma_R[FILTER_MAX_SIZE] = {1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
real darma_S[FILTER_MAX_SIZE] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
real darma_T[FILTER_MAX_SIZE] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

real comp_C_num[FILTER_MAX_SIZE] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
real comp_C_den[FILTER_MAX_SIZE - 1] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
real comp_F_num[FILTER_MAX_SIZE] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
real comp_F_den[FILTER_MAX_SIZE - 1] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};


// Local Variables ===================================================================
static ctrl_mode mode = CTRL_DISABLED;
static uint32_t ctrl_period_cycles;		// set update time, in cpu cycles
static float ctrl_period_sec;         // set update time, in seconds
static volatile uint32_t update_time = 0;		// set to the time the last update took, in cpu cycles
static volatile hist_data_t hist_data[HIST_SIZE];
static volatile uint32_t hist_head = 0;
static uint32_t hist_time_offset = 0;
//static char message[100] = "Hello, World";
static volatile float last_vel = 0;   // velocity chosen last update
static volatile int32_t last_encpos = 0.f;
static volatile real last_ctrl_out = 0.f;
//static volatile real ctrl_integrator = 0.f;   // control integrator variable.
static volatile real ff_target_pos_buf[FF_TARGETS];
static volatile real ff_target_vel_buf[FF_TARGETS];
static volatile uint32_t ff_target_head = 0;

// PID variables
static volatile float pid_i_sum = 0.f;

// Shared Filter variables (used by DARMA and comp)
static real filter_y_hist[FILTER_MAX_SIZE];   // output
static real filter_u_hist[FILTER_MAX_SIZE];   // input
static real filter_uc_hist[FILTER_MAX_SIZE];  // target (control input)
static uint32_t filter_head = 0;
static uint32_t filter_warmup = 0;             // counts the number of times darma has updated to make sure the buffers are full.

// Compensating filter variables
static real comp_c_hats[FILTER_MAX_SIZE];     // c_hats list (the past outputs from the C block -- see notes 6/11/14)
static real comp_f_hats[FILTER_MAX_SIZE];     // f_hats list (the past outputs from the F block)

// Function Predeclares ==============================================================
void set_update_cycles(uint32_t cycles);
real pid_ctrl(real dt, real target_pos, real target_vel, real encpos, real lastvel);
void bang_ctrl(real dt, real target_pos, real target_vel, real encpos);
real darma_ctrl();
real comp_ctrl();
bool fault_check(real encpos, real cmdpos, real *pos_error_deriv);

// Initializes the PIT timer used for control
void init_ctrl(void)
{
	// Clock up all of the PITs
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  PIT_TCTRL3 = PIT_TCTRL_TIE_MASK;
	
  NVIC_ENABLE_IRQ(IRQ_PIT_CH3);
	ctrl_set_period(1000);		// start with 1ms update frequency
  ctrl_enable(CTRL_DISABLED); // disable the timer which ctrl_set_period just enabled
	
	// Controller heartbeat (for checking clock regularity)
	//GPIOD_PDDR |= (1<<3);
	//PORTD_PCR3 = STANDARD_OUTPUT;

  // clear the history ringbuffer
  vmemset((void *)hist_data, 0, sizeof(hist_data_t) * HIST_SIZE);
  vmemset((void *)ff_target_pos_buf, 0, sizeof(real) * FF_TARGETS);
  vmemset((void *)ff_target_vel_buf, 0, sizeof(real) * FF_TARGETS);

  vmemset((void *)filter_u_hist, 0, sizeof(real) * FILTER_MAX_SIZE);
  vmemset((void *)filter_y_hist, 0, sizeof(real) * FILTER_MAX_SIZE);
  vmemset((void *)filter_uc_hist, 0, sizeof(real) * FILTER_MAX_SIZE);
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
    // turn off legacy contol mode (if it was enabled)
    old_stepper_mode = false;
    // need to clear PID variables to avoid major issues
    pid_i_sum = 0;
    get_enc_value(&last_encpos);
    last_vel = 0;
    //ctrl_integrator = 0;
    ff_target_head = 0;
    vmemset((void *)ff_target_pos_buf, 0, sizeof(real) * FF_TARGETS);
    vmemset((void *)ff_target_vel_buf, 0, sizeof(real) * FF_TARGETS);
    //||\\!! TODO: Re-fill the target pos buf with a first value?
    // if the mode has changed, reset the history buffer
    if(newmode != mode)
    {
      vmemset((void *)hist_data, 0, sizeof(hist_data_t) * HIST_SIZE);
      hist_head = 0;
      hist_time_offset = get_systick_tenus();   // so we don't have some 0's and then stuff way off in time at the same time
    }

    // reset filter history variables (used by darma and comp controllers)
    vmemset((void *)filter_u_hist, 0, sizeof(real) * FILTER_MAX_SIZE);
    vmemset((void *)filter_y_hist, 0, sizeof(real) * FILTER_MAX_SIZE);
    vmemset((void *)filter_uc_hist, 0, sizeof(real) * FILTER_MAX_SIZE);
    filter_head = 0;
    filter_warmup = 0;

    // darma history variables check
    if(CTRL_DARMA == newmode)
    {
      if(fabsf(darma_R[0]) < 1.e-6)   // somewhat arbitrary, but in my experience this is way too small to work.
      {
        hid_printf("'Very small value of R[0] means DARMA is not going to be stable! Please choose a larger R[0]");
        ctrl_enable(CTRL_DISABLED);
        return;
      }
    }

    // comp: clear additional filters
    if(CTRL_COMP == newmode)
    {
      vmemset((void *)comp_c_hats, 0, sizeof(real) * FILTER_MAX_SIZE);
      vmemset((void *)comp_f_hats, 0, sizeof(real) * FILTER_MAX_SIZE);
    }

    set_update_cycles(ctrl_period_cycles);

    //if(CTRL_BANG == newmode)
    //{
    // turn "off" the motor update period. Needed in case the step rate was very high.
    set_step_events_per_minute_ctrl(1);
    //}
  }
  else    // disable control
    PIT_TCTRL3 &= ~PIT_TCTRL_TEN_MASK;
  mode = newmode;
}

ctrl_mode ctrl_get_mode(void)
{
  return mode;
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
  hid_printf("%u\n", HIST_SIZE);   // # of entries we're going to print
  // the serial port can't take all this data at once, so we'll give it to them in bites...
  // We don't want to wait too long, however, so we'll put a timeout of 30ms on the transmits.
  for(uint32_t chunk = old_head_loc + 1; chunk < HIST_SIZE - 1; chunk += 100)
  {
    for(uint32_t i = 0; i < 3; i++)   // retry the packet up to 3 times.
    {
      delay_real(30);
      if(hid_print((void*)(hist_data + chunk), sizeof(hist_data_t) * min(HIST_SIZE - chunk, 100), 30))
        break;
    }
  }
  for(uint32_t chunk = 0; chunk < old_head_loc + 1; chunk += 100)
  {
    for(uint32_t i = 0; i < 3; i++)   // retry the packet up to 3 times.
    {
      delay_real(30);
      if(hid_print((void*)(hist_data + chunk), sizeof(hist_data_t) * min(old_head_loc + 1 - chunk, 100), 30))
        break;
    }
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
  real pos_error_deriv = 0.f;
	
	// Update the controller heartbeat
	//GPIOD_PTOR = (1<<3);

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

  // add target_pos and encpos to their respective filter ring buffers, and clear the current element of u for now.
  filter_head = (filter_head + 1) & (FILTER_MAX_SIZE - 1);     // power of 2 ring buffer.
  filter_u_hist[filter_head] = 0.f;
  filter_y_hist[filter_head] = encpos;
  filter_uc_hist[filter_head] = target_pos;
	
	// perform the control law
  switch(mode)
  {
  case CTRL_UNITY :
    if(pos_ctrl_mode)
      ctrl_out = target_pos;
    else
      ctrl_out = target_vel;
    break;
  case CTRL_PID :
    ctrl_out = pid_ctrl(ctrl_period_sec, target_pos, target_vel, encpos, last_vel);
    if(pos_ctrl_mode)
    {
      ctrl_out += ff_target_pos_buf[ff_target_head];   // use the advanced feedforward target here to null out the system lag.
    }
    break;
  case CTRL_BANG :
    bang_ctrl(ctrl_period_sec, target_pos, target_vel, encpos);
    ctrl_out = 0;   // bang-bang doesn't use velocity.
    break;
  case CTRL_DARMA :
    ctrl_out = darma_ctrl();    // darma_ctrl gets all the data it needs from the filter ringbuffers.
    break;
  case CTRL_COMP :
    ctrl_out = comp_ctrl();
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
    // check for faults - compare this target motor position with the actual motor position.
    if(fault_check(encpos, ctrl_out, &pos_error_deriv))
    {
      // We have a fault! Do something intelligent!!!! //||\\!!!
      //hid_printf("Fault detected!\n");
      //
    }
    ctrl_out = -((real)motorpos * enc_tics_per_step - ctrl_out) / ctrl_period_sec * 60;    // see notebook, 5/7/14
    //ctrl_out = -(encpos - ctrl_out) / ctrl_period_sec * 60;
  }

  // clamp the new velocity
  if(fabsf(ctrl_out) < min_ctrl_vel) ctrl_out = 0;
  if(fabsf(ctrl_out) > max_ctrl_vel) ctrl_out = copysignf(max_ctrl_vel, ctrl_out);

  // re-compute the control output after clamping for use by DARMA next time
  last_ctrl_out = ctrl_out * ctrl_period_sec / 60.f + (real)motorpos * enc_tics_per_step;
  filter_u_hist[filter_head] = last_ctrl_out;   // save for future use on filter buffer
  

  // save this update to the ring buffer
  //hist_head = (hist_head + 1) & (HIST_SIZE - 1);   // list_size is a power of 2, so list_size - 1 is 0b0..01..1
  if(++hist_head >= HIST_SIZE) hist_head = 0;
  hist_data[hist_head].time = time_of_update - hist_time_offset;  // rollover may occur here, but this is just reporting.
  hist_data[hist_head].motor_position = motorpos * enc_tics_per_step;
  hist_data[hist_head].position = encpos;
  hist_data[hist_head].target_pos = target_pos;
  hist_data[hist_head].target_vel = target_vel;
  //hist_data[hist_head].velocity = last_vel;
  hist_data[hist_head].pos_error_deriv = pos_error_deriv;
  hist_data[hist_head].cmd_velocity = ctrl_out;

  // write it out right now (testing!)
  //if(stream_ctrl_hist)
  //{
  //  usb_serial_write("$", 1);
  //  usb_serial_write(hist_data + hist_head, sizeof(hist_data_t));
  //}


  // the output was encoder tics per minute; we want that back in motor steps/minute
  ctrl_out = ctrl_out * steps_per_enc_tic;
  // set the new velocity
  if(mode != CTRL_BANG)
  {
    // don't do this when we're in bang mode...it does it internally.
    set_direction(ctrl_out > 0.f ? false : true);
    set_step_events_per_minute_ctrl((uint32_t)abs((int32_t)floorf(ctrl_out)));
  }
  
  // get the path target location (encoder tics) and velocity (encoder tics/minute) for the NEXT update (even including feedforward).
  // We will get the target advanced in time ctrl_feedforward_advance steps + 1 and keep it until it's current.
  ff_target_head = (ff_target_head + 1) & (FF_TARGETS - 1);   // advance the head
  path_get_target(ff_target_pos_buf + ff_target_head, ff_target_vel_buf + ff_target_head, time_of_update + (ctrl_feedforward_advance + 1) * ctrl_period_sec * TENUS_PER_SEC_F);

	
  
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
 //   //hid_printf("Old: %lu; New: %lu; Diff: %li; Target: %lu, Set: %lu\n", old_systic, new_systic, (long int)old_systic - (long int)new_systic, ctrl_period_cycles, ctrl_period_cycles - update_time);
 // }
	//else
 // {
	//	set_update_cycles(100);		// wait some minimum time before firing again
 //   //hid_printf("Old: %lu; New: %lu; Diff: %li; Target: %lu, Set: %lu\n", old_systic, new_systic, (long int)old_systic - (long int)new_systic, ctrl_period_cycles, 100);
 // }
 // 
 // //
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
      set_direction(false);
      //hid_printf("enc: %f target: %f diff: %f FORWARD\n", encpos, target_pos, encpos - target_pos);   // # of entries we're going to print
      //
    }
    else
    {
      set_direction(true);
      //hid_printf("enc: %f target: %f diff: %f BACKWARDS\n", encpos, target_pos, encpos - target_pos);   // # of entries we're going to print
      //
    }
    trigger_pulse();    // backdoor to fire a step RIGHT NOW
  }
}

// Darma controller
// computes the control law based on a DARMA-like controller (control law is based on current and
// past system inputs and outputs. That is, u(k) is computed from:
//  R*u(k) = T*uc(k) - S*y(k)
// where R, T, and S are polynomials in q^-1 (the backwards shift operator), specified in darma_[R/T/S].
// u(k) is the output of the controller/input of the system at the current time; uc(k) is the control input
// (reference input/target position) and y(k) is the current system output.
// This is designed to be used with either model-based control or model following control.
real darma_ctrl()
{
  real Ru = 0, Sy = 0, Tuc = 0, u_out;


  if(filter_warmup > FILTER_MAX_SIZE)   // don't run the controller until all buffers are full
  {

    // compile the terms. NOTE: Officially, the Ru term should only contain terms 1..end of R and u
    // (we will divide by R(0) in a minute to *obtain* u(k), the value to feed to the system). Instead
    // of dedicated logic, we set the first u in the hist buf (above) to 0.
    for(uint32_t i = 0, j = filter_head; i < FILTER_MAX_SIZE; ++i, j = (j - 1) & (FILTER_MAX_SIZE - 1))
    {
      // i indexes the coeficient masks, j indexes the history masks (and automatically loops
      Ru += darma_R[i] * filter_u_hist[j];
      Sy += darma_S[i] * filter_y_hist[j];
      Tuc += darma_T[i] * filter_uc_hist[j];
    }

    // compute the control law!
    u_out = 1/darma_R[0] * (Tuc - Sy - Ru);
  }
  else
  {
    u_out = filter_uc_hist[filter_head];   // unity mode control during warmup
    filter_warmup++;
  }

  return u_out;
}


// Compensating filter controller
// This is based on my notes 6/11/14-6/12/14. The controller is designed after a compensating network used by
// Matlab's sisotool. The structure looks like this:
//
//                     .-------.   f_hat
//         .---------->|   F   |--------.
//         |           *-------*        |
//    uc   |           .-------. c_hat  |   u     .--------.
//   ------+--->|+|--->|   C   |------>|+|------->| System |--+----> y (encoder read value)
//              -^     *-------*                  *--------*  |
//               |____________________________________________|
//
// C and F are both "compensating filters", user-selected causal linear IIR filters expressed in the form
//        b_0 + b_1 * z^-1 + ... + b_n * z^-n
//   C = -------------------------------------
//        1 + a_1 * z^-1 + ... + a^m * z^-m
//
// where the vectors b and a come from comp_C_num and comp_C_den, respectively, NOTE that comp_C_num[0] = b_0 
// while comp_C_den[0] = a_1!!
// 
// To compute the filters, I am using:
//      f_hat = F*uc  ---> f_hat * F_den = F_num * uc
// The first element of f_hat * F_den is f_hat * z^0 = f_hat[t]. This is what we are solving for.
// So, I compile the numerator term and the denominator term (except for the first element) and write:
//      f_hat[t] = F_num * uc + (f_hat * F_den)*
// where (f_hat * F_den)* is terms 2..m of the series.
//
// All the parameters needed for this function are already supplied in the filter tables.
//
real comp_ctrl(void)
{
  real ucFn, fhFd = 0., errCn, chCd = 0.;
  
  if(filter_warmup > FILTER_MAX_SIZE)   // don't run the controller until all buffers are full
  {

    // compile the filters. First element of both numerators is filled in by hand; first element of both denominators is what we're solving for.
    ucFn = comp_F_num[0] * filter_uc_hist[filter_head];
    errCn = comp_C_num[0] * (filter_uc_hist[filter_head] - filter_y_hist[filter_head]);    // error is uc - y.

    for(uint8_t i = 0, j = (filter_head - 1) & FILTER_MAX_SIZE; i < FILTER_MAX_SIZE - 1; i++, j = (j - 1) & FILTER_MAX_SIZE)
    {
      // i counts indexes in the filter polys; j counts indexes in the filter buffers.
      ucFn += comp_F_num[i + 1] * filter_uc_hist[j];
      fhFd += comp_F_den[i] * comp_f_hats[j];
      errCn += comp_C_num[i + 1] * (filter_uc_hist[j] - filter_y_hist[j]);
      chCd += comp_C_den[i] * comp_c_hats[j];
    }

    // compute the outputs
    comp_f_hats[filter_head] = ucFn - fhFd;
    comp_c_hats[filter_head] = errCn - chCd;
    return comp_f_hats[filter_head] + comp_c_hats[filter_head];
  }
  else
  {
    filter_warmup++;
    return filter_uc_hist[filter_head];   // unity control mode
  }
}

// Fault check
// Looks for significant changes in the error between the motor command position (coming out of the controller)
// and the actual encoder position. Returns true if a fault has been detected, and sets the pos_error_deriv parameter
// to the calculated error change.
bool fault_check(real encpos, real cmdpos, real *pos_error_deriv)
{
  static real last_cmdpos = -12345.f;
  static real last_pos_delta = -1.f;
  bool retval = false;

  if(last_cmdpos != -12345.f)    // first datapoint will be erronious
  {
    real pos_delta = fabsf(encpos - last_cmdpos);
    if(last_pos_delta != -1.f)
    {
      *pos_error_deriv = pos_delta - last_pos_delta;
      if(abs(*pos_error_deriv) > fault_thresh)
        retval = true;
    }
    last_pos_delta = pos_delta;
  }

  last_cmdpos = cmdpos;
  return retval;
}
