/*******************************************************************
 * IMC Controller USB Serial Control Interface - used for controlling
 * parameters and performing offline tests when not in an IMC network.
 * This codebase is merged with Matthew Sorenson's ME599 codebase, with 
 * ****SIGNIFICANT**** changes to pinnouts because of pins needed to interface
 * with the encoder (SPI). The ME599 codebase is retained with as few changes
 * as possible inside the ./imc folder. In general, when significant changes
 * were needed, hooks have been inserted into the IMC code which map back to
 * the local control code (see, for example, imc/stepper.c and stepper_hooks.c)
 *
 * Ben Weiss - 2014
 * University of Washington
 *
 * TODO:
 *  - implement velocity clamping before fault check so step inputs
 *    aren't interpreted as faults.
 *  - fix/re-enable the endstop cuttoffs in control_isr.c
 *  - Fix encoder reader losing track when in homing routine/
 *
 * Pinnout: This code is developed against a Teensy 3.1 MK20DX256 chip.
 *  0, 1 <--> PB16, PB17 - Min and Max limit switches
 *  2-5 <--> PD0, PA12, PA13, PD7 - I2C Address Offset (after Base Address) (pin 2 is bit 0)
 *  6-8 <--> PD4, PD2, PD3 - Motor Driver microstepping mode
 *  (disabled) 8 <--> PD3 - controller heartbeat (this bit toggles every time the controller updates)
 *  9 <--> PC3 - Stepper Disable
 *  SPI interface for SPI-reading encoder:
 *    10 <--> PC4 - CS
 *    11 <--> PC6 - DOUT
 *    12 <--> PC7 - DIN
 *    13 <--> PC5 - SCK
 *  15 <--> PC0 - Step
 *  16 <--> PB0 - Global sync line
 *  18 <--> PB3 - I2C SDA
 *  19 <--> PB2 - I2C SC
 *  20, 21, 22 <--> PD5, PD6, PC1 - SCK, DIN, CS of bitbanged SPI interface to encoder reader (when spienc.h:ENC_USE_SPIFIFO is not defined)
 *  22 <--> PC1 - Direction
 *  23 <--> PC2 - encoder index (neither used nor tested)
 *  25, 32 <--> PB19, PB18 - quadrature inputs
 *  
 *  The motor interface is described in in imc/hardware.h and imc/peripheral.h. NOTE that to accommodate SPI interface
 *  to the sensor, pinnouts for the motor driver have changed!
 *
 * Usage: The device enumerates as a USB serial adapter. The following commands are supported
 *  Mode commands:
 *   i - idle mode - no stepper control (driver disabled); encoder monitored
 *   f - fixed step - stepper rate (steps/min) set by next signed long integer. Successive (whitespace-separated)
 *       numbers can change the step size.
 *   m - move steps mode - moves a number of steps defined in the next signed long integer. Successive (whitespace-separated)
 *       numbers change the new number of "steps to go"
 *   n - IMC network control mode
 *   c* - Controller mode
 *       cp - PID control mode - target position (in encoder tics) set by next signed long integer. Successive
 *            (whitespace-separated) numbers change the target location.
 *       cu - Unity control mode - new velocity = target velocity at each update. Note that unity control doesn't do
 *            anything when presented with a step target (ps), because step paths leave velocity = 0.
 *       cb - Bang-Bang control mode - steps are directly fired based upon position error.
 *       cl - legacy (open-loop) control mode - IMC stock code is used. This is really implemented by turning off
 *            control and setting stepper_hooks.c:old_stepper_mode = true. This is only available when the system mode
 *            is IMC network control mode (n).
 *       cd - DARMA control mode. Uses a control structure derived from Astrom & Wittenmark's Self-Tuning Controller,
 *            (although the controller is not self-tuning in this implementation)
 *       cc - Compensating filter control mode. Uses a set of IIR digital filters in a feedback loop to enhance
 *            controller performance. This method will likely introduce lag, which can be compensated for by advancing
 *            the control signal.
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
 *       pr - random path mode. Generates a random path, keeping successive datapoints less than max vel (param a)
 *       pm - ramps-move mode. Move according to a trajectory generated using RAMPS's planner. Trajectory is
 *          specified using the pm parameter vector and executes only once. This path mode is used when the system mode is IMC
 *          network mode.
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
 *      kp* - PID control parameters
 *        kpp - PID proportional constant
 *        kpi - PID integral constant
 *        kpd - PID derivative constant
 *      km - control to position instead of velocity (int32 but represents a boolean - 1 means on, 0 means off)
 *      kf - feedforward time advance (in update steps - uint32)
 *      kt - fault detection threshhold - change in tics/update of the error between commanded position and actual position.
 *      ku - controller update period (in ms)
 *      kd* - DARMA control parameters
 *        kdr - DARMA control R vector. See notes in ctrl.c:darma_ctrl() for details
 *        kds - DARMA control S vector
 *        kdt - DARMA control T vector
 *      kc* - Compensating controller parameters
 *        kcn - C numerator - vector of coeficients for the C filter numerator (starting with z^0 and progressing towards z^-n)
 *        kcd - C denominator - vector of coefficients of the C filter denominator (starting with z^-1 and progressing towards z^-n)
 *        kco - F numerator - vector of coeficients for the F filter numerator (starting with z^0 and progressing towards z^-n)
 *        kcf - F denominator - vector of coefficients of the F filter denominator (starting with z^-1 and progressing towards z^-n)
 *    t - encoder tic count (int32)
 *    m* - motor parameters.
 *      mp - step position (int32)
 *      mf - force step counter reset whenever the step events per minute function is called. (int32 but represents a boolean - 1 means on, 0 means off)
 *    o - occasionally output encoder value. Value specifies the number of ms between reporting. 0 = off (uint)
 *    p* - path parameters
 *      pc - number of sines (1-5, int)
 *      pf - sine frequency base - rad/tenus
 *      pa - sine amplitude (tics)
 *      pr - random path move amplitude (0->1 scalar, normalized against ctrl max vel)
 *      pm - path parameters used when executing a ramps-style move. This is a vector, with elements 
 *             {length, total_length, initial_rate, nominal_rate, final_rate, acceleration}. All elements are int32_t type.
 *             For this vector, all distances are in motor steps and all times are in minutes.
 *    q - encoder tics per step (float)
 *    s - Stream control history in real time. Boolean (0 = false, 1 = true)
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

#include "stepper_hooks.h"
#include "param_hooks.h"
#include "qdenc.h"
#include "ctrl.h"
#include "path.h"

#include "imc/hardware.h"
#include "imc/main_imc.h"
#include "imc/stepper.h"
#include "imc/parameters.h"
#include "imc/utils.h"
#include "imc/peripheral.h"

typedef enum {
  RL_IDLE,
  RL_MANUAL,      // manually moves the motor.
  RL_CTRL,        // control enabled, but target trajectory is specified over serial
  RL_IMC          // IMC mode - listening to i2c bus for instructions. Implies controller is active and path is in ramps mode
} runlevel_e;

#define USB_RX_TIMEOUT     2    // ms to wait for a packet before continuing the idle loop
#define USB_INPUT_BUF_SIZE 150  // characters.

// Global Variables ==========================================================
//extern volatile uint32_t systick_millis_count;    // system millisecond timer
extern float pid_kp, pid_ki, pid_kd;
extern float max_ctrl_vel, min_ctrl_vel;
extern bool pos_ctrl_mode;
extern uint32_t ctrl_feedforward_advance;
extern float sine_freq_base, sine_amp, rand_scale;
extern uint32_t sine_count;
extern bool force_steps_per_minute;
extern float fault_thresh;
extern bool old_stepper_mode;
extern real darma_R[FILTER_MAX_SIZE];
extern real darma_S[FILTER_MAX_SIZE];
extern real darma_T[FILTER_MAX_SIZE];
extern real comp_C_num[FILTER_MAX_SIZE];
extern real comp_C_den[FILTER_MAX_SIZE - 1];
extern real comp_F_num[FILTER_MAX_SIZE];
extern real comp_F_den[FILTER_MAX_SIZE - 1];
extern bool stream_ctrl_hist;

char message[200];
runlevel_e runlevel = RL_IDLE;
enum { M_FIXED_SPEED, M_MOVE_STEPS } manual_mode;

float enc_tics_per_step = 21.7343;//1.4986;                       // encoder tics per motor (micro)step (roughly)
float steps_per_enc_tic = 1/21.7343;//1/1.4986;                          // = 1 / enc_tics_per_step

volatile uint32_t systick_tenus_count;     // millisecond counter which updates every SYSTICK_UPDATE_MS ms.
volatile uint32_t csr_last;



// Local Variables ===========================================================
static char msg_build[30];
static uint32_t show_encoder_time = 0;
static bool moving = false;
static int32_t ramps_move_params[6] = {0, 0, 0, 0, 0, 0};
static char usb_input_buffer[USB_INPUT_BUF_SIZE];
static uint32_t input_buf_len = 0;

// Function Predeclares ======================================================
int get_usb_line(char *dest, uint32_t dest_size);
void parse_usb();
void parse_ctrl_msg(const char *buf, uint32_t *i, uint32_t count);
void parse_path_msg(const char *buf, uint32_t *i, uint32_t count);
void parse_get_param(const char *buf, uint32_t *i, uint32_t count);
void parse_set_param(const char *buf, uint32_t *i, uint32_t count);
void set_enc_tics_per_step(float etps);


// This hook is called by main at the beginning of setup.
int main()
{
  uint32_t next_encoder_time = 0;
  int32_t value;

  // change the cpu systic clock to only roll over every SYSTICK_UPDATE_MS milliseconds:
  SYST_RVR = (F_CPU / 1000) * (SYSTICK_UPDATE_TEN_US / 100L) - 1;
  systick_millis_count = 0;

  hid_init(read_i2c_address());

  //reset_hardware();
  //initialize_stepper_state();
  enc_Init();
  // enable motor and wait for a bit for the coils to stabilize
  //enable_stepper();

  imc_init();     // initialize the IMC controller

  // turn on the max endstop for testing
  {
    msg_set_param_t msg;
    msg.param_id = IMC_PARAM_MAX_LIMIT_EN;
    msg.param_value = 1;
    handle_set_parameter(&msg);
    msg.param_id = IMC_PARAM_MIN_LIMIT_PULLUP;
    msg.param_value = IMC_PULLUP;
    handle_set_parameter(&msg);
    msg.param_id = IMC_PARAM_MAX_LIMIT_PULLUP;
    msg.param_value = IMC_PULLUP;
    handle_set_parameter(&msg);
    msg.param_id = IMC_PARAM_MAX_LIMIT_INV;
    msg.param_value = 1;
    handle_set_parameter(&msg);
    msg.param_id = IMC_PARAM_MIN_LIMIT_INV;
    msg.param_value = 0;
    handle_set_parameter(&msg);
  }

  // set up the stepper hooks into the IMC module
  init_stepper_hooks();
  init_param_hooks();

  delay_real(100);

	// start up control
	init_ctrl();
  set_enc_value(0);   // zero out the encoder


  while(1)
  {
    if(RL_IMC == runlevel)
      imc_idle();   // IMC main loop

    if(hid_available() > 0)
    {
      parse_usb();
    }
    //||\\!! Just for testing
    hid_flush(5);

    
    if(show_encoder_time > 0 && get_systick_tenus() > next_encoder_time)
	  {
      next_encoder_time = get_systick_tenus() + show_encoder_time * 100;
      if(get_enc_value(&value))
        hid_printf("'%li**\n", value);   // signal we lost track!
      else
        hid_printf("'%li\n", value);
      //usb_serial_write(message,strlen(message));
    }
    
    if(RL_MANUAL == runlevel && M_MOVE_STEPS == manual_mode && get_steps_to_go() == -1 && moving)
    {
      // done with move!
      get_enc_value(&value);
      hid_printf("'Move complete. New step position = %li; Encoder position = %li\n", (long)get_motor_position(), (long)value);
      //usb_serial_write(message,strlen(message));
      moving = false;
    }
#ifndef USE_QD_ENC
    enc_idle();
#endif
  }
}

// This code assumes we won't get more than one command per packet!!
int get_usb_line(char *dest, uint32_t dest_size)
{
  uint32_t count;
  // packets are not null-terminated!
  //Collect packets until we see a zero character...since our packets are now small, it is possible some commands
  // (especially vector commands) may not complete in a single packet.

  for(;;) {
    if(input_buf_len + input_buf_len > USB_INPUT_BUF_SIZE)
    {
      // this is too long a line! Ignore it and get out of here.
      input_buf_len = 0;
      return 0;
    }
    count = hid_read((uint8_t*)usb_input_buffer + input_buf_len, USB_RX_TIMEOUT);
    if(!count)    // no (complete) packet read; return 0 but keep the partial packet for next time.
      return 0;

    input_buf_len += HID_PACKLEN;
    //hid_printf("Got a packet %02X %02X %02X %02X!\n", usb_input_buffer[0], usb_input_buffer[1], usb_input_buffer[2], usb_input_buffer[3]);

    // check for zero characters --> we have a complete command.
    for(uint32_t i = 0; i < HID_PACKLEN; i++)
      if(0 == usb_input_buffer[input_buf_len - HID_PACKLEN + i])
      {
        // complete string found. Send this off and clear the buffer.
        memcpy((void*)dest, (void*)usb_input_buffer, min(input_buf_len - HID_PACKLEN + i + 1, dest_size - 1));
        dest[dest_size-1] = 0;    // just in case the input buffer was too long.
        input_buf_len = 0;
        //hid_printf("Got a line: '%s'\n", dest);
        return 1;
      }
  }
}

// Parses the serial input
void parse_usb(void)
{
  char buf[USB_INPUT_BUF_SIZE];
  uint32_t count = 0;
  int read = 0;
  int32_t foo, foo2;
  uint32_t i;
  
  if(!get_usb_line(buf, USB_INPUT_BUF_SIZE))
    return;   // nothing to be done.

  count = strlen(buf);    // get_usb_line guarantees null-terminate strings.
  
  for(i = 0; i < count; i++)
  {
    switch(buf[i++])
    {
    case 'i':
      // idle state
      runlevel = RL_IDLE;
      ctrl_enable(CTRL_DISABLED);
      old_stepper_mode = false;
      stop_motion();
      set_steps_to_go(-1);
      moving = false;
      disable_stepper();
      
      hid_printf("'Idle mode\n");

      break;
    case 'f':
      runlevel = RL_MANUAL;
      manual_mode = M_FIXED_SPEED;
      old_stepper_mode = false;
      // try to read a step frequency
      if(sscanf(buf + i, " %li%n", (long *)&foo, &read) == 1)
      {
        i += read;
        set_direction(foo < 0);
        set_step_events_per_minute_ctrl((uint32_t)fabsf(foo));
      }
      hid_printf("'Fixed mode. Rate: %li steps/min\n", (long)(get_direction() ? -1 : 1) * (long)get_step_events_per_minute());
      
      ctrl_enable(CTRL_DISABLED);
      enable_stepper();
      set_steps_to_go(-1);
      start_moving();
      moving = true;
      break;
    case 'n':   // IMC network mode
      runlevel = RL_IMC;
      hid_printf("'IMC network mode\n");
      
      // turn on a controller, if there isn't one already
      if(ctrl_get_mode() == CTRL_DISABLED && !old_stepper_mode)
        ctrl_enable(CTRL_UNITY);
      // set path to ramps mode
      get_enc_value(&foo2);
      path_imc((real)foo2);

      enable_stepper();
      set_steps_to_go(-1);
      start_moving();
      moving = true;

      enter_sync_state();   // tell the stepper module to float the sync line, signaling we're ready for a move.
      break;
    case 'm':
      // move a specified number of steps.
      old_stepper_mode = false;
      // try to read a number of steps to go.
      if(sscanf(buf + i, " %li%n", (long *)&foo, &read) == 1)
      {
        i += read;
        if(0 == foo) foo = 1;
        ctrl_enable(CTRL_DISABLED);
        set_direction(foo < 0);
        set_step_events_per_minute_ctrl(10000);
        set_steps_to_go((int32_t)labs((long)foo));
        
        runlevel = RL_MANUAL;
        manual_mode = M_MOVE_STEPS;
        get_enc_value(&foo2);
        hid_printf(message, "'Move Steps mode. Moving from %li by %li steps\n'  Current encoder value = %li\n", get_motor_position(), foo, foo2);

        enable_stepper();
        start_moving();
        moving = true;
      }
      else
      {
        hid_printf("'Couldn't parse a distance to move!\n");
        runlevel = RL_IDLE;
        ctrl_enable(CTRL_DISABLED);
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
      switch(runlevel)
      {
      case RL_MANUAL:
        if(M_FIXED_SPEED == manual_mode)
        {
          // try to read a step frequency
          if(sscanf(buf + i - 1, "%li%n", (long *)&foo, &read) == 1)
          {
            i += read;
            set_direction(foo < 0);
            set_step_events_per_minute_ctrl((uint32_t)abs(foo));
            hid_printf("'Moving at %li steps/min\n", (long)(get_direction() ? -1 : 1) * (long)get_step_events_per_minute());
          }
          else
          {
            hid_printf("'Didn't understand new f value\n");
          }
        }
        else
        {
          // try to read a new move target
          if(sscanf(buf + i - 1, "%li%n", (long *)&foo, &read) == 1)
          {
            i += read;
            set_direction((foo) < 0);
            set_steps_to_go((uint32_t)abs(foo));
            get_enc_value(&foo2);
            hid_printf("'Move Steps mode. Moving from %li by %li steps\n  Current encoder value = %li\n", get_motor_position(), foo, foo2);
            
            start_moving();
            moving = true;
          }
        }
        break;
      case RL_CTRL:
        // try to read a new move target
        if(sscanf(buf + i - 1, "%li%n", (long *)&foo, &read) == 1)
        {
          i += read;
        
          path_set_step_target(foo);

          get_enc_value(&foo2);
          hid_printf("'Stepping from %li to %li\n", foo2, foo);
          
        }
        break;
      case RL_IDLE :
      case RL_IMC :
        // nothing to do.
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
      hid_printf("'PID control mode. Step path from %li to %li\n", foo, foo2);
      
    }
    else
    {
      // don't change the path mode unless we got a value.
      hid_printf("'PID control mode.\n");
      
    }
    if(runlevel < RL_CTRL)    // don't kick us out of imc mode if we're in it.
      runlevel = RL_CTRL;
    ctrl_enable(CTRL_PID);

    enable_stepper();
    start_moving();
    moving = true;
    break;
  case 'u':
    {
      // Unity control mode
      // measure the time it takes to read the encoder, for reference.

      uint32_t old_systic = SYST_CVR, new_systic, update_time;

      get_enc_value(&foo);

	    new_systic = SYST_CVR;

	    if(new_systic < old_systic)		// counter counts down!
		    update_time = old_systic - new_systic;
	    else	// counter rolled over
		    update_time = old_systic - new_systic + SYST_RVR;

      
      foo = get_motor_position() * enc_tics_per_step;
      path_set_step_target(foo);
      
      if(runlevel < RL_CTRL)    // don't kick us out of imc mode if we're in it.
        runlevel = RL_CTRL;
      ctrl_enable(CTRL_UNITY);

      enable_stepper();
      start_moving();
      moving = true;

      hid_printf("'Unity control mode.\n'Encoder read time: %f ms\n", (float)update_time * 1000.f / (float)F_BUS);
      
      break;
    }
  case 'b':
    // Bang control mode
    get_enc_value(&foo);
    path_set_step_target(foo);
    
    if(runlevel < RL_CTRL)    // don't kick us out of imc mode if we're in it.
      runlevel = RL_CTRL;
    ctrl_enable(CTRL_BANG);

    enable_stepper();
    start_moving();
    moving = true;

    hid_printf("'Bang-bang control mode.\n");
    
    break;
  case 'l':
    // Legacy control mode
    // only available if system mode is IMC network mode.
    if(RL_IMC == runlevel)
    {
      // We implement this by disabling the control module and turning
      // on stepper_hooks.c:old_stepper_mode.
      ctrl_enable(CTRL_DISABLED);

      old_stepper_mode = true;

      moving = true;

      hid_printf("'Legacy control mode.\n");
      
    }
    else
    {
      // complain
      hid_printf("'Legacy control mode can't be used when not in IMC mode!\n");
      
    }
    break;
  case 'd':
    // Darma control mode
    
    get_enc_value(&foo);

    path_set_step_target(foo);
    
    if(runlevel < RL_CTRL)    // don't kick us out of imc mode if we're in it.
      runlevel = RL_CTRL;
    ctrl_enable(CTRL_DARMA);

    enable_stepper();
    start_moving();
    moving = true;

    hid_printf("'DARMA control mode.\n");
    
    
    break;
  case 'c':
    // Compensating filter control mode
    
    get_enc_value(&foo);

    path_set_step_target(foo);
    
    if(runlevel < RL_CTRL)    // don't kick us out of imc mode if we're in it.
      runlevel = RL_CTRL;
    ctrl_enable(CTRL_COMP);

    enable_stepper();
    start_moving();
    moving = true;

    hid_printf("'Compensating control mode.\n");
    
    break;
  default :
    hid_printf("'Unrecognized command.\n");
    
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
      hid_printf("'Step path mode. Stepping from %li to %li\n", foo2, foo);
      
    }
    else
    {
      get_enc_value(&foo);

      hid_printf("'Step path mode.\n");
      
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
        hid_printf("'Failed to parse path element!\n");
        
      }
      break;
    case 's':   // pcs
      path_custom_start();
      break;
    case 'c':   // pcc
      path_custom_clear();
      break;
    default :
      hid_printf("'Unrecognized command.\n");
      
      break;
    }
    break;
  case 'q':   // pq - sine mode
    path_sines_start();
    break;
  case 'r':   // pr - random mode
    path_rand_start();
    break;
  case 'm':   // pm - ramps-style move
    {
      // Set up the move
      msg_queue_move_t msg;
      msg.length = ramps_move_params[0];
      msg.total_length = ramps_move_params[1];
      msg.initial_rate = ramps_move_params[2];
      msg.nominal_rate = ramps_move_params[3];
      msg.final_rate = ramps_move_params[4];
      msg.acceleration = ramps_move_params[5];
      get_enc_value(&foo);
      path_imc(foo);
      path_ramps_move(&msg);
    }
    break;

  default :
    hid_printf("'Unrecognized command.\n");
    
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
    hid_printf("%f\n", max_ctrl_vel);
    break;
  case 'i':
    // minimum control value
    hid_printf("%f\n", min_ctrl_vel);
    break;
  case 't':
    // encoder tic count
    if(!get_enc_value(&foo))
    {
      hid_printf("%li\n", (long)foo); //
    }
    else
    {
      hid_printf("%li\n'Lost Track!\n", (long)foo); //
    }
    
    break;
  case 'm':
    // motor driver parameters
    switch(buf[(*i)++])
    {
    case 'f':
      // mf - force counter reset on update
      hid_printf("%li\n", (long)force_steps_per_minute);
      break;
    case 'p':
      // mp - motor step position
      hid_printf("%li\n", (long)get_motor_position());
      break;
    }
    break;

  case 'f':
    // current move frequency
    hid_printf("%li\n", (long)(get_direction() ? -1 : 1) * (long)get_step_events_per_minute());
    break;
  case 'o':
    // current show encoder frequency
    hid_printf("%lu\n", (unsigned long)show_encoder_time);
    break;
  case 'k':
    // Control parameters
    switch(buf[(*i)++])
    {
    case 'p':
      // PID control parameters
      switch(buf[(*i)++])
      {
      case 'p':
        // kpp - proportional constant
        hid_printf("%f\n", pid_kp);
        break;
      case 'i':
        // kpi - integral constant
        hid_printf("%f\n", pid_ki);
        break;
      case 'd':
        // kpd - derivative constant
        hid_printf("%f\n", pid_kd);
        break;
      }
      break;
    case 'm':
      // km - control to position mode
      hid_printf("%i\n", pos_ctrl_mode ? 1 : 0);
      break;
    case 'f':
      // kf - feedforward advance time (update steps)
      hid_printf("%lu\n", ctrl_feedforward_advance);
      break;
    case 't':
      // fault threshold
      hid_printf("%f\n", fault_thresh);
      break;
    case 'u' :
      // controller update period (in ms)
      hid_printf("%f\n", ctrl_get_period() / 1000.f);
      break;
    case 'd':
      {
        hid_printf("'Get started.\n");
        
        // DARMA control parameters
        real *target = NULL;
        int32_t m = 0;
        switch(buf[(*i)++])
        {
        case 'r':
          // kdr - R vector
          target = darma_R;
          break;
        case 's':
          // kds - S vector
          target = darma_S;
          break;
        case 't':
          // kdt - T vector
          target = darma_T;
          break;
        default :
          // didn't understand
          hid_printf("Unknown command.\n");
          
          break;
        }
        if(target)
        {
          for(m = FILTER_MAX_SIZE - 1; m > 0; m--)
            if(0.f != target[m])
              break;
          hid_printf("'m= %i.\n", (int)m);
          message[0] = 0;
          for(uint32_t k = 0; k <= m; k++)
          {
            sprintf(msg_build, "%f ", target[k]);
            strcat(message, msg_build);
          }
          strcat(message, "\n");
          hid_print(message, strlen(message), 100);
        }
        hid_printf("'get complete\n");
      }
      break;
      
      case 'c':
      {
        // Compensating control parameters
        real *target = NULL;
        uint8_t m = FILTER_MAX_SIZE - 1;
        switch(buf[(*i)++])
        {
        case 'n':
          // kcn - C numerator vector
          target = comp_C_num;
          break;
        case 'd':
          // kcd - C denominator vector
          target = comp_C_den;
          m--;
          break;
        case 'o':
          // kco - F numerator vector
          target = comp_F_num;
          break;
        case 'f':
          // kcf - F denominator vector
          target = comp_F_den;
          m--;
          break;
        }
        if(target)
        {
          for(; m > 0; m--)
            if(0.f != target[m])
              break;
          //hid_printf("'m= %i.\n", (int)m);
          message[0] = 0;
          for(uint32_t k = 0; k <= m; k++)
          {
            sprintf(msg_build, "%f ", target[k]);
            strcat(message, msg_build);
          }
          strcat(message, "\n");
          hid_print(message, strlen(message), 100);
        }
        //hid_printf("'get complete\n");
      }
      break;

    }
    break;

  case 'u':
    // last controller update time
    hid_printf("%f\n", ctrl_get_update_time());
    break;
  case 'q':
    // encoder tics per step (float)
    hid_printf("%f\n", enc_tics_per_step);
    break;
  case 's':
    // stream ctrl history
    hid_printf("%i\n", (int)stream_ctrl_hist);
    break;
  case 'p':
    // path mode variables:
    switch(buf[(*i)++])
    {
    case 'c':
      // pc - sine count
      hid_printf("%lu\n", sine_count);
      break;
    case 'f':
      // pf - sine frequency base (hz)
      hid_printf("%f\n", sine_freq_base);
      break;
    case 'a':
      // pa - sine amplitude
      hid_printf("%f\n", sine_amp);
      break;
    case 'r':
      // pr - random move scale
      hid_printf("%f\n", rand_scale);
      break;
    case 'm':
      // pm - ramps-style move parameters
      message[0] = 0;
      for(uint32_t k = 0; k < 6; k++)
      {
        sprintf(msg_build, "%li ", ramps_move_params[k]);
        strcat(message, msg_build);
      }
      strcat(message, "\n");
      hid_print(message, strlen(message), 100);
    }
    break;
  case 'd':
    // controller history dump (binary)
    output_history();
    break;
  default :
    // didn't understand!
    hid_printf("'I didn't understand which parameter you want to query.\n");
  }
}

bool read_float(const char * buf, uint32_t *i, float *value)
{
  uint32_t read;
  float ffoo;
  if(sscanf(buf + *i, " %f%n", &ffoo, (int*)&read) == 1)
  {
    *i += read;
    *value = ffoo;
    return true;
  }
  return false;
}

bool read_int(const char * buf, uint32_t *i, int32_t *value)
{
  uint32_t read;
  int32_t foo;
  if(sscanf(buf + *i, " %li%n", &foo, (int*)&read) == 1)
  {
    *i += read;
    *value = foo;
    return true;
  }
  return false;
}

bool read_uint(const char * buf, uint32_t *i, uint32_t *value)
{
  uint32_t read;
  uint32_t foo;
  if(sscanf(buf + *i, " %lu%n", &foo, (int*)&read) == 1)
  {
    *i += read;
    *value = foo;
    return true;
  }
  return false;
}

bool read_vector_float(const char *buf, uint32_t *i, float *vector, uint32_t size)
{
  uint32_t read;
  if(vector)
  {
    // clear the buffer
    vmemset((void *)vector, 0, sizeof(real) * size);
    // load the buffer
    for(uint32_t k = 0; k < size; k++)
    {
      if(sscanf(buf + *i, " %f%n", &vector[k], (int*)&read) == 1)
        *i += read;
      else
        break;    // we're done!
    }
    return true;
  }
  return false;
}

bool read_vector_int(const char *buf, uint32_t *i, int32_t *vector, uint32_t size)
{
  uint32_t read;
  if(vector)
  {
    // clear the buffer
    vmemset((void *)vector, 0, sizeof(int32_t) * size);
    // load the buffer
    for(uint32_t k = 0; k < size; k++)
    {
      if(sscanf(buf + *i, " %li%n", &vector[k], (int*)&read) == 1)
        *i += read;
      else
        break;    // we're done!
    }
    return true;
  }
  return false;
}


// Parses a Set Parameter message
void parse_set_param(const char * buf, uint32_t *i, uint32_t count)
{
  int32_t foo;
  float ffoo;
  bool parseok = false;
  // which parameter?
  switch(buf[(*i)++])
  {
  case 'a':
    // maximum ctrl velocity
    parseok = read_float(buf, i, &max_ctrl_vel);
    break;
  case 'i':
    // minimum ctrl velocity
    parseok = read_float(buf, i, &min_ctrl_vel);
    break;
    
  case 't':
    // encoder tic count
    parseok = read_int(buf, i, &foo);
    set_enc_value(foo);
    break;
    
  case 'm':
    // Motor driver (stepper.c) parameters
    switch(buf[(*i)++])
    {
    case 'f':
      // mf - force motor timer update
      parseok = read_int(buf, i, &foo); 
      force_steps_per_minute = (foo != 0);
      break;
    case 'p':
      // mp - Motor position
      parseok = read_int(buf, i, &foo); 
      set_motor_position(foo);
    }
    break;
    
  case 'o':
    // encoder readout frequency
    parseok = read_uint(buf, i, &show_encoder_time);
    break;
  case 'f':
    // current move frequency
    if(RL_MANUAL == runlevel)
    {
      parseok = read_int(buf, i, &foo);
      if(M_FIXED_SPEED == manual_mode)
        set_direction(foo < 0);
      set_step_events_per_minute_ctrl((uint32_t)abs(foo));
    }
    else
    {
      hid_printf("'Cannot set step frequency when not in Fixed Step mode or Move Steps mode!\n");
      
      parseok = true;
    }
    break;
  case 'k':
    // Control parameters
    switch(buf[(*i)++])
    {
    case 'p':
      // PID control parameters
      switch(buf[(*i)++])
      {
      case 'p':
        // kpp - proportional constant
        parseok = read_float(buf, i, &pid_kp);
        break;
      case 'i':
        // kpi - integral constant
        parseok = read_float(buf, i, &pid_ki);
        break;
      case 'd':
        // kpd - derivative constant
        parseok = read_float(buf, i, &pid_kd);
        break;
      }
      break;
    case 'm':
      // km - control to position mode
      parseok = read_uint(buf, i, (uint32_t*)&foo);
      pos_ctrl_mode = (foo != 0);
      break;
    case 'f':
      // kf - feedforward advance steps
      parseok = read_uint(buf, i, &ctrl_feedforward_advance);
      break;
    case 't':
      // kt - fault threshold
      parseok = read_float(buf, i, &ffoo);
      fault_thresh = fabsf(ffoo);
      break;
    case 'u':
      // ku - controller update period (ms)
      parseok = read_float(buf, i, &ffoo);
      ctrl_set_period((uint32_t)(ffoo * 1000.f));
      break;
    case 'd':
      // DARMA control parameters
      switch(buf[(*i)++])
      {
      case 'r':
        // kdr - R vector
        parseok = read_vector_float(buf, i, darma_R, FILTER_MAX_SIZE);
        break;
      case 's':
        // kpi - integral constant
        parseok = read_vector_float(buf, i, darma_S, FILTER_MAX_SIZE);
        break;
      case 't':
        // kpd - derivative constant
        parseok = read_vector_float(buf, i, darma_T, FILTER_MAX_SIZE);
        break;
      }
      break;

    case 'c':
      // Compensating control parameters
      switch(buf[(*i)++])
      {
      case 'n':
        // kcn - C numerator vector
        parseok = read_vector_float(buf, i, comp_C_num, FILTER_MAX_SIZE);
        break;
      case 'd':
        // kcd - C denominator vector
        parseok = read_vector_float(buf, i, comp_C_den, FILTER_MAX_SIZE - 1);
        break;
      case 'o':
        // kco - F numerator vector
        parseok = read_vector_float(buf, i, comp_F_num, FILTER_MAX_SIZE);
        break;
      case 'f':
        // kcf - F denominator vector
        parseok = read_vector_float(buf, i, comp_F_den, FILTER_MAX_SIZE - 1);
        break;
      }
      break;

    }
    break;
  case 'q':
    // encoder tics per step
    parseok = read_float(buf, i, &ffoo);
    set_enc_tics_per_step(ffoo);
    break;
  case 's':
    // stream ctrl history
    parseok = read_int(buf, i, &foo);
    stream_ctrl_hist = (foo != 0);
    hid_printf("Streaming Set: %i", (int)stream_ctrl_hist);
    break;
  case 'p':
    // Path sine mode parameters
    switch(buf[(*i)++])
    {
    case 'c':
      // pc - sine count
      parseok = read_uint(buf, i, &sine_count);
      break;
    case 'f':
      // pf - sine freq
      parseok = read_float(buf, i, &ffoo);
      path_sines_setfreq(ffoo);
      break;
    case 'a':
      // pa - sine amplitude
      parseok = read_float(buf, i, &sine_amp);
      break;
    case 'r':
      // pr - random scale
      parseok = read_float(buf, i, &rand_scale);
      break;
    case 'm':
      // pm - ramps-style move parameters
      parseok = read_vector_int(buf, i, ramps_move_params, 6);
      break;
    }
    break;
  default :
    // didn't understand!
    hid_printf("'I didn't understand which parameter you want to query.\n");
    
    parseok = true;
  }
  if(!parseok)
  {
    hid_printf("'Failed to parse new value.\n");
  }
}

// Random number generator:
uint32_t rand_uint32 (void)
{
   static uint32_t z1 = 12345, z2 = 12345, z3 = 12345, z4 = 12345;
   uint32_t b;
   b  = ((z1 << 6) ^ z1) >> 13;
   z1 = ((z1 & 4294967294U) << 18) ^ b;
   b  = ((z2 << 2) ^ z2) >> 27; 
   z2 = ((z2 & 4294967288U) << 2) ^ b;
   b  = ((z3 << 13) ^ z3) >> 21;
   z3 = ((z3 & 4294967280U) << 7) ^ b;
   b  = ((z4 << 3) ^ z4) >> 12;
   z4 = ((z4 & 4294967168U) << 13) ^ b;
   return (z1 ^ z2 ^ z3 ^ z4);
}


// sets the encoder tics per motor step parameter (and its inverse)
void set_enc_tics_per_step(float etps)
{
  enc_tics_per_step = etps;
  steps_per_enc_tic = 1.f / (float)etps;
}

// We are orverriding the systic ISR provided by Teensy because it now counts
// in 10ms increments instead of 1ms. To keep compatibility with delay(), which
// imc needs, I have reverted this to the stock version.
void systick_isr(void)
{
  //systick_tenus_count += SYSTICK_UPDATE_TEN_US;
  systick_millis_count++;
}