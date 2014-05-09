/*******************************************************************
 * Encoder Stepper Test
 * Ben Weiss - 2014
 * University of Washington
 *
 * Purpose: Interfaces with both a stepper motor and an encoder, with modes for different
 *   ways of controlling the stepper (feedback or no).
 *
 * Pinnout: This code is developed against a Teensy 3.1 MK20DX256 chip.
 *  PB18, PB19 <--> 32, 25 - quadrature inputs
 *  PC2 <--> 23 - encoder index (neither used nor tested)
 *  SPI interface for SPI-reading encoder:
 *    PC4 <--> 10 - CS
 *    PC6 <--> 11 - DOUT
 *    PC7 <--> 12 - DIN
 *    PC5 <--> 13 - SCK
 *  PD3 <--> 8 - controller heartbeat (this bit toggles every time the controller updates)
 *  The motor interface is described in in hardware.h. NOTE that to accommodate SPI interface
 *  to the sensor, pinnouts for the motor driver have changed!
 *
 * Usage: The device enumerates as a USB serial adapter. The following commands are supported
 *  Mode commands:
 *   i - idle mode - no stepper control (driver disabled); encoder monitored
 *   f - fixed step - stepper rate (steps/min) set by next signed long integer. Successive (whitespace-separated)
 *       numbers can change the step size.
 *   m - move steps mode - moves a number of steps defined in the next signed long integer. Successive (whitespace-separated)
 *       numbers change the new number of "steps to go"
 *   c* - Controller mode
 *       cp - PID control mode - target position (in encoder tics) set by next signed long integer. Successive
 *            (whitespace-separated) numbers change the target location.
 *       cu - Unity control mode - new velocity = target velocity at each update. Note that unity control doesn't do
 *            anything when presented with a step target (ps), because step paths leave velocity = 0.
 *       cb - Bang-Bang control mode - steps are directly fired based upon position error.
 *
 *   p* - target trajectory mode - 
 *       ps - step mode. Successive whitespace-separated inputs are taken to be new position targets.
 *       pc* - custom path following mode. Allows the controller to follow a custom profile. Load the custom
 *            profile using pcp(ush); start the move with pcs(tart), and clear the custom path buffer with pcc(lear)
 *          pcp - push a new move onto the custom path buffer. Format: pcp TTT XXX VVV where TTT (uint32) is the time to reach
 *                this position (since move start, in ms). TTT must be greater than the previous step. XXX (float) is the 
 *                location target, and VVV (float) is the velocity target, in encoder tics and encoder tics/min, respectively.
 *          pcs - starts executing the custom path buffer. The controller target will be set by linearly interpolating
 *                between the entries pushed to the buffer with pcp, counting time from the moment this command was received.
 *          pcc - clears the custom path buffer.
 *       pq - sine mode. Generates position and velocity targets from five summed sine waves.
 * 
 *  Parameter commands:
 *   gX - gets parameter X's value
 *   sX YYYY - sets parameter X's value to YYYY
 * 
 *  Parameters:
 *    a - mAximum velocity allowed for controller output. Any velocity output by the controller above this value clamps to this value.
 *    d - controller history dump (binary)
 *    f - current move frequency (in fixed mode, this is the last number entered) (int32)
 *    i - mInimum velocity allowed for controller output. Any velocity output by the controller below this value clamps to 0.
 *    k* - Controller parameters:
 *      kp - PID proportional constant
 *      ki - PID integral constant
 *      kd - PID derivative constant
 *      km - control to position instead of velocity (int32 but represents a boolean - 1 means on, 0 means off)
 *      kf - feedforward time advance (in update steps - uint32)
 *    t - encoder tic count (int32)
 *    m* - motor parameters.
 *      mp - step position (int32)
 *      mf - force step counter reset whenever the step events per minute function is called. (int32 but represents a boolean - 1 means on, 0 means off)
 *    o - occasionally output encoder value. Value specifies the number of ms between reporting. 0 = off (uint)
 *    p - controller update period (in ms)
 *    q - encoder tics per step (float)
 *    s* - sine path mode parameters:
 *      sc - number of sines (1-5, int)
 *      sf - sine frequency base - rad/tenus
 *      sa - sine amplitude (tics)
 *    u - last controller update time (in ms), read only
 *
 *  Note: Responses meant to be human-readible (i.e. Debug strings for ctrl_design_gui) start with an apostrophe (')
 *
 *
 * License: This code is for internal development only and has not been licensed for public release.
 *  (c) 2014 University of Washington
 *
 *******************************************************************/

#include "common.h"

#include "stepper.h"
#include "hardware.h"
#include "qdenc.h"
#include "ctrl.h"
#include "path.h"

typedef enum {
  SS_IDLE,
  SS_FIXED_SPEED,
  SS_MOVE_STEPS,
  SS_CTRL       // any of the controllers
} sys_state_e;

// Global Variables ==========================================================
//extern volatile uint32_t systick_millis_count;    // system millisecond timer
extern float pid_kp, pid_ki, pid_kd;
extern float max_ctrl_vel, min_ctrl_vel;
extern bool pos_ctrl_mode;
extern uint32_t ctrl_feedforward_advance;
extern float sine_freq_base, sine_amp;
extern uint32_t sine_count;
extern bool force_steps_per_minute;
sys_state_e sysstate = SS_IDLE;
float enc_tics_per_step = 1.4986;                       // encoder tics per motor (micro)step (roughly)
float steps_per_enc_tic = 1/1.4986;                          // = 1 / enc_tics_per_step

volatile uint32_t systick_tenus_count;     // millisecond counter which updates every SYSTICK_UPDATE_MS ms.
volatile uint32_t csr_last;
volatile bool systick_rollover_handled = false;   // flag to tell the isr we already trapped this one.


// Local Variables ===========================================================
static char message[100] = "Hello, World";
static uint32_t show_encoder_time = 0;
static bool moving = false;

// Function Predeclares ======================================================
void parse_usb();
void parse_ctrl_msg(const char *buf, uint32_t *i, uint32_t count);
void parse_path_msg(const char *buf, uint32_t *i, uint32_t count);
void parse_get_param(const char *buf, uint32_t *i, uint32_t count);
void parse_set_param(const char *buf, uint32_t *i, uint32_t count);
void set_enc_tics_per_step(float etps);


int main()
{
  uint32_t next_encoder_time = 0;
  int32_t value;

  // change the cpu systic clock to only roll over every SYSTICK_UPDATE_MS milliseconds:
  SYST_RVR = (F_CPU / 1000) * (SYSTICK_UPDATE_TEN_US / 100L) - 1;

  // reset the prioritization of interrupts on our chip. We'll set everything down to 3, then
  // boost up the control interrupt to 2 AND the stepper interrupts to 1. 
  // IRQ priorities are set using the high nibble.
  for(uint32_t i = 0; i < NVIC_NUM_INTERRUPTS; i++)
  {
    NVIC_SET_PRIORITY(i, 48);
  }
  NVIC_SET_PRIORITY(IRQ_PIT_CH3, 32);
  NVIC_SET_PRIORITY(IRQ_PIT_CH0, 16);
  NVIC_SET_PRIORITY(IRQ_PIT_CH1, 16);

  
  reset_hardware();
  initialize_stepper_state();
  enc_Init();
  // enable motor and wait for a bit for the coils to stabilize
  enable_stepper();
  delay_real(100);
  set_enc_value(0);   // zero out the encoder
	// start up control
	init_ctrl();
  

  while(1){

    if(usb_serial_available() > 0)
    {
      parse_usb();
    }

    
    if(show_encoder_time > 0 && get_systick_tenus() > next_encoder_time)
		{
      next_encoder_time = get_systick_tenus() + show_encoder_time * 100;
      if(get_enc_value(&value))
        sprintf(message, "'%li**\n", value);   // signal we lost track!
      else
        sprintf(message, "'%li\n", value);
      usb_serial_write(message,strlen(message));
    }
    
    if(SS_MOVE_STEPS == sysstate && get_steps_to_go() == -1 && moving)
    {
      // done with move!
      get_enc_value(&value);
      sprintf(message, "'Move complete. New step position = %li; Encoder position = %li\n", (long)get_motor_position(), (long)value);
      usb_serial_write(message,strlen(message));
      moving = false;
    }
#ifndef USE_QD_ENC
    enc_idle();
#endif
  }
}

// Parses the serial input
void parse_usb(void)
{
  char buf[100];
  uint32_t count = 0;
  int read = 0;
  int32_t foo, foo2;
  uint32_t i;
  
  count = usb_serial_read(buf, min(usb_serial_available(), 100));
  // packets are not null-terminated!
  if(count < 100)
    buf[count] = '\0';
  for(i = 0; i < count; i++)
  {
    switch(buf[i++])
    {
    case 'i':
      // idle state
      sysstate = SS_IDLE;
      ctrl_enable(CTRL_DISABLED);
      stop_motion();
      set_steps_to_go(-1);
      moving = false;
      disable_stepper();
      
      sprintf(message, "'Idle mode\n");
      usb_serial_write(message, strlen(message));

      break;
    case 'f':
      sysstate = SS_FIXED_SPEED;
      // try to read a step frequency
      if(sscanf(buf + i, " %li%n", (long *)&foo, &read) == 1)
      {
        sprintf(message, "'Got a %li\nBuf is %s\nread = %i\n", foo);
        usb_serial_write(message, strlen(message));
        i += read;
        set_direction(foo > 0);
        set_step_events_per_minute((uint32_t)fabsf(foo));
      }
      sprintf(message, "'Fixed mode. Rate: %li steps/min\n", (long)(get_direction() ? 1 : -1) * (long)get_step_events_per_minute());
      usb_serial_write(message, strlen(message));
      enable_stepper();
      set_steps_to_go(-1);
      execute_move();
      moving = true;
      break;
    case 'm':
      // move a specified number of steps.
      // try to read a number of steps to go.
      if(sscanf(buf + i, " %li%n", (long *)&foo, &read) == 1)
      {
        i += read;
        if(0 == foo) foo = 1;
        set_direction(foo > 0);
        set_step_events_per_minute(10000);
        set_steps_to_go(abs(foo));
        
        sysstate = SS_MOVE_STEPS;
        get_enc_value(&foo2);
        sprintf(message, "'Move Steps mode. Moving from %li by %li steps\n'  Current encoder value = %li\n", get_motor_position(), foo, foo2);
        usb_serial_write(message, strlen(message));
        enable_stepper();
        execute_move();
        moving = true;
      }
      else
      {
        sprintf(message, "'Couldn't parse a distance to move!\n");
        usb_serial_write(message, strlen(message));
        sysstate = SS_IDLE;
        moving = false;
      }
      break;
    case 'c':
      // Control mode
      parse_ctrl_msg(buf, &i, count);
      break;
    case 'p':
      // path control commands
      parse_path_msg(buf, &i, count);
      break;
    case 'g':
      // get a parameter value
      parse_get_param(buf, &i, count);
      break;
    case 's':
      // set a parameter value
      parse_set_param(buf, &i, count);
      break;
    default :
      // try to read a new step frequency or target location given previous state
      switch(sysstate)
      {
      case SS_FIXED_SPEED:
        // try to read a step frequency
        if(sscanf(buf + i - 1, "%li%n", (long *)&foo, &read) == 1)
        {
          i += read;
          set_direction(foo > 0);
          set_step_events_per_minute((uint32_t)abs(foo));
          sprintf(message, "'Moving at %li steps/min\n", (long)(get_direction() ? 1 : -1) * (long)get_step_events_per_minute());
          usb_serial_write(message, strlen(message));
        }
        else
        {
          sprintf(message, "'Didn't understand new f value\n");
          usb_serial_write(message, strlen(message));
        }
        break;
      case SS_MOVE_STEPS:
        // try to read a new move target
        if(sscanf(buf + i - 1, "%li%n", (long *)&foo, &read) == 1)
        {
          i += read;
          set_direction((foo) > 0);
          set_steps_to_go((uint32_t)abs(foo));
          get_enc_value(&foo2);
          sprintf(message, "'Move Steps mode. Moving from %li by %li steps\n  Current encoder value = %li\n", get_motor_position(), foo, foo2);
          usb_serial_write(message, strlen(message));
          execute_move();
          moving = true;
        }
        break;
      case SS_CTRL:
        // try to read a new move target
        if(sscanf(buf + i - 1, "%li%n", (long *)&foo, &read) == 1)
        {
          i += read;
        
          path_set_step_target(foo);

          get_enc_value(&foo2);
          sprintf(message, "'Stepping from %li to %li\n", foo2, foo);
          usb_serial_write(message, strlen(message));
        }
        break;
      }
    }
  }
}

// Parses a Control mode message
void parse_ctrl_msg(const char * buf, uint32_t *i, uint32_t count)
{
  int32_t foo, foo2;
  int read = 0;
  // which parameter?
  switch(buf[(*i)++])
  {
  case 'p':
    // PID control mode
    // try to read a step target
    if(sscanf(buf + *i, " %li%n", (long *)&foo, &read) == 1)
    {
      *i += read;

      path_set_step_target(foo);

      get_enc_value(&foo2);
      sprintf(message, "'PID control mode. Step path from %li to %li\n", foo, foo2);
      usb_serial_write(message, strlen(message));
    }
    else
    {
      // don't change the path mode unless we got a value.
      sprintf(message, "'PID control mode.\n");
      usb_serial_write(message, strlen(message));
    }
    sysstate = SS_CTRL;
    ctrl_enable(CTRL_PID);

    enable_stepper();
    execute_move();
    moving = true;
    break;
  case 'u':
    // Unity control mode
    get_enc_value(&foo);
    path_set_step_target(foo);
    sysstate = SS_CTRL;
    ctrl_enable(CTRL_UNITY);

    enable_stepper();
    execute_move();
    moving = true;

    sprintf(message, "'Unity control mode.\n");
    usb_serial_write(message, strlen(message));
    break;
  case 'b':
    // Bang control mode
    get_enc_value(&foo);
    path_set_step_target(foo);
    sysstate = SS_CTRL;
    ctrl_enable(CTRL_BANG);

    enable_stepper();
    execute_move();
    moving = true;

    sprintf(message, "'Bang-bang control mode.\n");
    usb_serial_write(message, strlen(message));
    break;
  default :
    sprintf(message, "'Unrecognized command.\n");
    usb_serial_write(message, strlen(message));
    break;
  }
}

// Parses a Path mode message
void parse_path_msg(const char * buf, uint32_t *i, uint32_t count)
{
  int32_t foo, foo2;
  custom_path_dp_t dp;
  int read = 0;
  // which parameter?
  switch(buf[(*i)++])
  {
  case 's':   // ps
    // Step mode
    // try to read a step target
    if(sscanf(buf + *i, " %li%n", (long *)&foo, &read) == 1)
    {
      *i += read;

      get_enc_value(&foo2);
      sprintf(message, "'Step path mode. Stepping from %li to %li\n", foo2, foo);
      usb_serial_write(message, strlen(message));
    }
    else
    {
      get_enc_value(&foo);

      sprintf(message, "'Step path mode.\n");
      usb_serial_write(message, strlen(message));
    }
    path_set_step_target(foo);
    
    break;
  case 'c':     //pc
    // Custom path following mode
    switch(buf[(*i)++])
    {
    case 'p':   // pcp
      // try to read three parameters
      if(sscanf(buf + *i, " %lu %f %f%n", (unsigned long *)&dp.time, &dp.target_pos, &dp.target_vel, &read) == 3)
      {
        *i += read;

        dp.time = dp.time * 100;  // from ms to tenus

        path_custom_add_elem(&dp);
      }
      else
      {
        sprintf(message, "'Failed to parse path element!\n");
        usb_serial_write(message, strlen(message));
      }
      break;
    case 's':   // pcs
      path_custom_start();
      break;
    case 'c':   // pcc
      path_custom_clear();
      break;
    default :
      sprintf(message, "'Unrecognized command.\n");
      usb_serial_write(message, strlen(message));
      break;
    }
    break;
  case 'q':   // pq - sine mode
    path_sines_start();
    break;
  default :
    sprintf(message, "'Unrecognized command.\n");
    usb_serial_write(message, strlen(message));
    break;
  }
}

// Parses a Get Parameter message
void parse_get_param(const char * buf, uint32_t *i, uint32_t count)
{
  int32_t foo;
  // which parameter?
  switch(buf[(*i)++])
  {
  case 'a':
    // maximum control value
    sprintf(message, "%f\n", max_ctrl_vel);
    usb_serial_write(message, strlen(message));
    break;
  case 'i':
    // minimum control value
    sprintf(message, "%f\n", min_ctrl_vel);
    usb_serial_write(message, strlen(message));
    break;
  case 't':
    // encoder tic count
    get_enc_value(&foo);
    sprintf(message, "%li\n", (long)foo);
    usb_serial_write(message, strlen(message));
    break;
  case 'm':
    // motor driver parameters
    switch(buf[(*i)++])
    {
    case 'f':
      // mf - force counter reset on update
      sprintf(message, "%li\n", (long)force_steps_per_minute);
      usb_serial_write(message, strlen(message));
      break;
    case 'p':
      // mp - motor step position
      sprintf(message, "%li\n", (long)get_motor_position());
      usb_serial_write(message, strlen(message));
      break;
    }
    break;

  case 'f':
    // current move frequency
    sprintf(message, "%li\n", (long)(get_direction() ? 1 : -1) * (long)get_step_events_per_minute());
    usb_serial_write(message, strlen(message));
    break;
  case 'o':
    // current show encoder frequency
    sprintf(message, "%lu\n", (unsigned long)show_encoder_time);
    usb_serial_write(message, strlen(message));
    break;
  case 'k':
    // PID control parameters
    switch(buf[(*i)++])
    {
    case 'p':
      // kp - proportional constant
      sprintf(message, "%f\n", pid_kp);
      usb_serial_write(message, strlen(message));
      break;
    case 'i':
      // ki - integral constant
      sprintf(message, "%f\n", pid_ki);
      usb_serial_write(message, strlen(message));
      break;
    case 'd':
      // kd - derivative constant
      sprintf(message, "%f\n", pid_kd);
      usb_serial_write(message, strlen(message));
      break;
    case 'm':
      // km - control to position mode
      sprintf(message, "%i\n", pos_ctrl_mode ? 1 : 0);
      usb_serial_write(message, strlen(message));
      break;
    case 'f':
      // kf - feedforward advance time (update steps)
      sprintf(message, "%lu\n", ctrl_feedforward_advance);
      usb_serial_write(message, strlen(message));
      break;
    }
    break;

  case 'u':
    // last controller update time
    sprintf(message, "%f\n", ctrl_get_update_time());
    usb_serial_write(message, strlen(message));
    break;
  case 'p':
    // controller update period (in ms)
    sprintf(message, "%f\n", ctrl_get_period() / 1000.f);
    usb_serial_write(message, strlen(message));
    break;
  case 'q':
    // encoder tics per step (float)
    sprintf(message, "%f\n", enc_tics_per_step);
    usb_serial_write(message, strlen(message));
    break;
  case 's':
    // sine path mode variables:
    switch(buf[(*i)++])
    {
    case 'c':
      // sc - sine count
      sprintf(message, "%lu\n", sine_count);
      usb_serial_write(message, strlen(message));
      break;
    case 'f':
      // sf - sine frequency base (hz)
      sprintf(message, "%f\n", sine_freq_base);
      usb_serial_write(message, strlen(message));
      break;
    case 'a':
      // sa - sine amplitude
      sprintf(message, "%f\n", sine_amp);
      usb_serial_write(message, strlen(message));
      break;
    }
    break;
  case 'd':
    // controller history dump (binary)
    output_history();
    break;
  default :
    // didn't understand!
    sprintf(message, "'I didn't understand which parameter you want to query.\n");
    usb_serial_write(message, strlen(message));
  }
}

// Parses a Set Parameter message
void parse_set_param(const char * buf, uint32_t *i, uint32_t count)
{
  int32_t foo;
  float ffoo;
  int read;
  bool parseok = false;
  // which parameter?
  switch(buf[(*i)++])
  {
  case 'a':
    // maximum ctrl velocity
    if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
    {
      *i += read;
      max_ctrl_vel = ffoo;
      parseok = true;
    }
    break;
  case 'i':
    // minimum ctrl velocity
    if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
    {
      *i += read;
      min_ctrl_vel = ffoo;
      parseok = true;
    }
    break;
    
  case 't':
    // encoder tic count
    if(sscanf(buf + *i, " %li%n", (long *)&foo, &read) == 1)
    {
      *i += read;
      set_enc_value(foo);
      parseok = true;
    }
    break;
    
  case 'm':
    // Motor driver (stepper.c) parameters
    switch(buf[(*i)++])
    {
    case 'f':
      // mf - force motor timer update
      if(sscanf(buf + *i, " %li%n", &foo, &read) == 1)
      {
        *i += read;
        force_steps_per_minute = (foo != 0);
        parseok = true;
      }
      break;
    case 'p':
      // mp - Motor position
      if(sscanf(buf + *i, " %li%n", (long *)&foo, &read) == 1)
      {
        *i += read;
        set_motor_position(foo);
        parseok = true;
      }
      break;
    }
    break;
    
  case 'o':
    // encoder readout frequency
    if(sscanf(buf + *i, " %lu%n", (long *)&foo, &read) == 1)
    {
      *i += read;
      show_encoder_time = foo;
      parseok = true;
    }
    break;
  case 'f':
    // current move frequency
    if(SS_FIXED_SPEED == sysstate || SS_MOVE_STEPS == sysstate)
    {
      // try to read a step frequency
      if(sscanf(buf + *i, " %li%n", (long *)&foo, &read) == 1)
      {
        *i += read;
        if(SS_FIXED_SPEED == sysstate)
          set_direction(foo > 0);
        set_step_events_per_minute((uint32_t)abs(foo));
        parseok = true;
      }
    }
    else
    {
      sprintf(message, "'Cannot set step frequency when not in Fixed Step mode or Move Steps mode!\n");
      usb_serial_write(message, strlen(message));
      parseok = true;
    }
    break;
  case 'k':
    // PID control parameters
    switch(buf[(*i)++])
    {
    case 'p':
      // kp - proportional constant
      if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
      {
        *i += read;
        pid_kp = ffoo;
        parseok = true;
      }
      break;
    case 'i':
      // ki - integral constant
      if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
      {
        *i += read;
        pid_ki = ffoo;
        parseok = true;
      }
      break;
    case 'd':
      // kd - derivative constant
      if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
      {
        *i += read;
        pid_kd = ffoo;
        parseok = true;
      }
      break;
    case 'm':
      // kd - derivative constant
      if(sscanf(buf + *i, " %lu%n", &foo, &read) == 1)
      {
        *i += read;
        pos_ctrl_mode = (foo != 0);
        parseok = true;
      }
      break;
    case 'f':
      // kf - feedforward advance steps
      if(sscanf(buf + *i, " %lu%n", &foo, &read) == 1)
      {
        *i += read;
        ctrl_feedforward_advance = foo;
        parseok = true;
      }
      break;
    }
    break;
  case 'p':
    // controller update period (ms)
    if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
    {
      *i += read;
      ctrl_set_period((uint32_t)(ffoo * 1000.f));
      parseok = true;
    }
    break;
  case 'q':
    // encoder tics per step
    if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
    {
      *i += read;
      set_enc_tics_per_step(ffoo);
      parseok = true;
    }
    break;
  case 's':
    // Path sine mode parameters
    switch(buf[(*i)++])
    {
    case 'c':
      // sc - sine count
      if(sscanf(buf + *i, " %li%n", &foo, &read) == 1)
      {
        *i += read;
        sine_count = foo;
        parseok = true;
      }
      break;
    case 'f':
      // sf - sine freq
      if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
      {
        *i += read;
        path_sines_setfreq(ffoo);
        parseok = true;
      }
      break;
    case 'a':
      // sa - sine amplitude
      if(sscanf(buf + *i, " %f%n", &ffoo, &read) == 1)
      {
        *i += read;
        sine_amp = ffoo;
        parseok = true;
      }
      break;
    }
    break;
  default :
    // didn't understand!
    sprintf(message, "'I didn't understand which parameter you want to query.\n");
    usb_serial_write(message, strlen(message));
    parseok = true;
  }
  if(!parseok)
  {
    sprintf(message, "'Failed to parse new value.\n");
    usb_serial_write(message, strlen(message));
  }
}


// sets the encoder tics per motor step parameter (and its inverse)
void set_enc_tics_per_step(float etps)
{
  enc_tics_per_step = etps;
  steps_per_enc_tic = 1.f / (float)etps;
}

// We are orverriding the systic ISR provided by Teensy because it now counts
// in 10ms increments instead of 1ms.
void systick_isr(void)
{
  systick_tenus_count += SYSTICK_UPDATE_TEN_US;
}