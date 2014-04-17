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
 *   p - PID control mode - target position (in encoder tics) set by next signed long integer. Successive
 *       (whitespace-separated) numbers change the target location.
 * 
 *  Parameter commands:
 *   gX - gets parameter X's value
 *   sX YYYY - sets parameter X's value to YYYY
 * 
 *  Parameters:
 *    d - controller history dump (binary)
 *    f - current move frequency (in fixed mode, this is the last number entered) (int32)
 *    k - Controller parameters:
 *      kp - PID proportional constant
 *      ki - PID integral constant
 *      kd - PID derivative constant
 *    t - encoder tic count (int32)
 *    m - motor step position (int32)
 *    o - occasionally output encoder value. Value specifies the number of ms between reporting. 0 = off (uint)
 *    p - controller update period (in ms)
 *    u - last controller update time (in ms), read only
 *
 *
 * License: This code is for internal development only and has not been licensed for public release.
 *  (c) 2014 University of Washington
 *
 *******************************************************************/

#include "common.h"
#include <usb_serial.h>
#include <stdio.h>
#include <string.h>

#include "stepper.h"
#include "hardware.h"
#include "qdenc.h"
#include "ctrl.h"
#include "path.h"

typedef enum {
  SS_IDLE,
  SS_FIXED_SPEED,
  SS_MOVE_STEPS,
  SS_PID_CTRL
} sys_state_e;

// Global Variables ==========================================================
extern volatile uint32_t systick_millis_count;    // system millisecond timer
extern float pid_kp, pid_ki, pid_kd;
sys_state_e sysstate = SS_IDLE;
uint32_t enc_tics_per_step;                       // encoder tics per motor (micro)step (roughly)
float steps_per_enc_tic;                          // = 1 / enc_tics_per_step

// this only rolls over every 50 days of execution time, so I'll ignore that possibility.

// Local Variables ===========================================================
static char message[100] = "Hello, World";
static uint32_t show_encoder_time = 0;
static bool moving = false;


// Function Predeclares ======================================================
void parse_usb();
void parse_get_param(const char *buf, uint32_t *i, uint32_t count);
void parse_set_param(const char *buf, uint32_t *i, uint32_t count);
void set_enc_tics_per_step(uint32_t etps);



int main()
{
  uint32_t next_encoder_time = 0;
  int32_t value;
  
  //PORTC_PCR5 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  //GPIOC_PDDR |= 1<<5;
  reset_hardware();
  initialize_stepper_state();
  enc_Init();
  // enable motor and wait for a bit for the coils to stabilize
  enable_stepper();
  delay(100);
  set_enc_value(0);   // zero out the encoder
	// start up control
	init_ctrl();
  while(1){
		// are there serial bytes to read?
		if(usb_serial_available() > 0)
		{
			parse_usb();
    }
    
    if(show_encoder_time > 0 && systick_millis_count > next_encoder_time)
		{
      next_encoder_time = systick_millis_count + show_encoder_time;
      if(get_enc_value(&value))
        sprintf(message, "%li**\n", value);   // signal we lost track!
      else
        sprintf(message, "%li\n", value);
      usb_serial_write(message,strlen(message));
    }
    
    if(SS_MOVE_STEPS == sysstate && get_steps_to_go() == -1 && moving)
    {
      // done with move!
      get_enc_value(&value);
      sprintf(message, "Move complete. New step position = %li; Encoder position = %li\n", (long)get_motor_position(), (long)value);
      usb_serial_write(message,strlen(message));
      moving = false;
    }
    enc_idle();
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
      stop_motion();
      set_steps_to_go(-1);
      moving = false;
      
      sprintf(message, "Idle mode\n");
      usb_serial_write(message, strlen(message));
      break;
    case 'f':
      sysstate = SS_FIXED_SPEED;
      // try to read a step frequency
      if(sscanf(buf + i, " %li%n", (long *)&foo, &read))
      {
        i += read;
        set_direction(foo > 0);
        set_step_events_per_minute((uint32_t)abs(foo));
      }
      sprintf(message, "Fixed mode. Rate: %li steps/min\n", (long)(get_direction() ? 1 : -1) * (long)get_step_events_per_minute());
      usb_serial_write(message, strlen(message));
      enable_stepper();
      set_steps_to_go(-1);
      execute_move();
      moving = true;
      break;
    case 'm':
      // move a specified number of steps.
      // try to read a number of steps to go.
      if(sscanf(buf + i, " %li%n", (long *)&foo, &read))
      {
        i += read;
        set_direction(foo > 0);
        set_step_events_per_minute(10000);
        set_steps_to_go(abs(foo));
        
        sysstate = SS_MOVE_STEPS;
        get_enc_value(&foo);
        sprintf(message, "Move Steps mode. Moving from %li by %li steps\n  Current encoder value = %li\n", get_motor_position(), foo, foo);
        usb_serial_write(message, strlen(message));
        enable_stepper();
        execute_move();
        moving = true;
      }
      else
      {
        sprintf(message, "Couldn't parse a distance to move!\n");
        usb_serial_write(message, strlen(message));
        sysstate = SS_IDLE;
        moving = false;
      }
      break;
    case 'p':
      // PID control mode
      // try to read a step target
      if(sscanf(buf + i, " %li%n", (long *)&foo, &read))
      {
        i += read;
        
        path_set_step_target(foo);
        sysstate = SS_PID_CTRL;
        ctrl_enable(CTRL_PID);

        enable_stepper();
        execute_move();
        moving = true;

        get_enc_value(&foo2);
        sprintf(message, "PID control mode. Stepping from %li to %li\n", foo, foo2);
        usb_serial_write(message, strlen(message));
      }
      else
      {
        get_enc_value(&foo);
        path_set_step_target(foo);
        sysstate = SS_PID_CTRL;
        ctrl_enable(CTRL_PID);

        enable_stepper();
        execute_move();
        moving = true;

        sprintf(message, "PID control mode.\n");
        usb_serial_write(message, strlen(message));
      }
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
        if(sscanf(buf + i, " %li%n", (long *)&foo, &read))
        {
          i += read;
          set_direction(foo > 0);
          set_step_events_per_minute((uint32_t)abs(foo));
        }
        break;
      case SS_MOVE_STEPS:
        // try to read a new move target
        if(sscanf(buf + i, " %li%n", (long *)&foo, &read))
        {
          i += read;
          set_direction((foo) > 0);
          set_steps_to_go((uint32_t)abs(foo));
          get_enc_value(&foo2);
          sprintf(message, "Move Steps mode. Moving from %li by %li steps\n  Current encoder value = %li\n", get_motor_position(), foo, foo2);
          usb_serial_write(message, strlen(message));
          execute_move();
          moving = true;
        }
        break;
      }
    }
  }
}

// Parses a Get Parameter message
void parse_get_param(const char * buf, uint32_t *i, uint32_t count)
{
  int32_t foo;
  // which parameter?
  switch(buf[(*i)++])
  {
  case 't':
    // encoder tic count
    get_enc_value(&foo);
    sprintf(message, "Encoder Tic Count: %li\n", (long)foo);
    usb_serial_write(message, strlen(message));
    break;
  case 'm':
    // motor step position
    sprintf(message, "Motor Position (steps): %li\n", (long)get_motor_position());
    usb_serial_write(message, strlen(message));
    break;
  case 'f':
    // current move frequency
    sprintf(message, "Move Freq: %li\n", (long)(get_direction() ? 1 : -1) * (long)get_step_events_per_minute());
    usb_serial_write(message, strlen(message));
    break;
  case 'o':
    // current show encoder frequency
    sprintf(message, "Encoder every: %lu\n", (unsigned long)show_encoder_time);
    usb_serial_write(message, strlen(message));
    break;
  case 'k':
    // PID control parameters
    switch(buf[(*i)++])
    {
    case 'p':
      // kp - proportional constant
      sprintf(message, "Kp: %f\n", pid_kp);
      usb_serial_write(message, strlen(message));
      break;
    case 'i':
      // ki - integral constant
      sprintf(message, "Ki: %f\n", pid_ki);
      usb_serial_write(message, strlen(message));
      break;
    case 'd':
      // kd - derivative constant
      sprintf(message, "Kd: %f\n", pid_kd);
      usb_serial_write(message, strlen(message));
      break;
    }
  case 'u':
    // last controller update time
    sprintf(message, "Last ctrl update time: %f ms\n", ctrl_get_update_time());
    usb_serial_write(message, strlen(message));
    break;
  case 'p':
    // controller update period (in ms)
    sprintf(message, "Controller update period: %lu ms\n", ctrl_get_period() / 1000.f);
    usb_serial_write(message, strlen(message));
    break;
  case 'd':
    // controller history dump (binary)
    output_history();
    break;
  default :
    // didn't understand!
    sprintf(message, "I didn't understand which parameter you want to query.\n");
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
  case 't':
    // encoder tic count
    if(sscanf(buf + *i, " %li%n", (long *)&foo, &read))
    {
      *i += read;
      set_enc_value(foo);
      parseok = true;
    }
    break;
    
  case 'm':
    // motor step position
    if(sscanf(buf + *i, " %li%n", (long *)&foo, &read))
    {
      *i += read;
      set_motor_position(foo);
      parseok = true;
    }
    break;
  case 'o':
    // encoder readout frequency
    if(sscanf(buf + *i, " %lu%n", (long *)&foo, &read))
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
      if(sscanf(buf + *i, " %li%n", (long *)&foo, &read))
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
      sprintf(message, "Cannot set step frequency when not in Fixed Step mode or Move Steps mode!\n");
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
      if(sscanf(buf + *i, " %f%n", &ffoo, &read))
      {
        *i += read;
        pid_kp = foo;
        parseok = true;
      }
      break;
    case 'i':
      // ki - integral constant
      if(sscanf(buf + *i, " %f%n", &ffoo, &read))
      {
        *i += read;
        pid_ki = foo;
        parseok = true;
      }
      break;
    case 'd':
      // kd - derivative constant
      if(sscanf(buf + *i, " %f%n", &ffoo, &read))
      {
        *i += read;
        pid_kd = foo;
        parseok = true;
      }
      break;
    }
  case 'p':
    // controller update period (ms)
    if(sscanf(buf + *i, " %lu%n", &foo, &read))
    {
      *i += read;
      ctrl_set_period(foo * 1000L);
      parseok = true;
    }
    break;
  default :
    // didn't understand!
    sprintf(message, "I didn't understand which parameter you want to query.\n");
    usb_serial_write(message, strlen(message));
    parseok = true;
  }
  if(!parseok)
  {
    sprintf(message, "Failed to parse new value.\n");
    usb_serial_write(message, strlen(message));
  }
}


// sets the encoder tics per motor step parameter (and its inverse)
void set_enc_tics_per_step(uint32_t etps)
{
  enc_tics_per_step = etps;
  steps_per_enc_tic = 1.f / (float)etps;
}