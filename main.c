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
 *  PC2 <-->23 - encoder index (neither used nor tested)
 *  The motor interface is described in in hardware.h
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
 *    t - encoder tic count (int32)
 *    m - motor step position (int32)
 *    f - current move frequency (in fixed mode, this is the last number entered) (int32)
 *    p,i,d - PID controller's P, I, and D constants, respectively (int32)
 *    k - encoder tics per motor step (float)
 *    o - occasionally output encoder value. Value specifies the number of ms between reporting. 0 = off (uint)
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

typedef enum {
  SS_IDLE,
  SS_FIXED_SPEED,
  SS_MOVE_STEPS,
  SS_PID_CTRL
} sys_state_e;

// Global Variables ==========================================================
extern volatile uint32_t systick_millis_count;    // system millisecond timer
// this only rolls over every 50 days of execution time, so I'll ignore that possibility.

// Local Variables ===========================================================
static char message[100] = "Hello, World";
sys_state_e sysstate = SS_IDLE;
uint32_t show_encoder_time = 0;
bool moving = false;


// Function Predeclares ======================================================
void parse_usb();
void parse_get_param(const char *buf, uint32_t *i, uint32_t count);
void parse_set_param(const char *buf, uint32_t *i, uint32_t count);
extern void ftm2_isr(void);
void QEI_Init(void);



int main()
{
  uint32_t next_encoder_time = 0;
  
  //PORTC_PCR5 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  //GPIOC_PDDR |= 1<<5;
  reset_hardware();
  initialize_stepper_state();
  QEI_Init();
  // enable motor and wait for a bit for the coils to stabilize
  enable_stepper();
  delay(100);
  set_enc_value(0);   // zero out the encoder
  while(1){
		// are there serial bytes to read?
		if(usb_serial_available() > 0)
		{
			parse_usb();
    }
    
    if(show_encoder_time > 0 && systick_millis_count > next_encoder_time)
		{
      next_encoder_time = systick_millis_count + show_encoder_time;
      sprintf(message, "%li--%lu--%lu\n", get_enc_value(), isr1_count, isr2_count);
      usb_serial_write(message,strlen(message));
    }
    
    if(SS_MOVE_STEPS == sysstate && get_steps_to_go() == -1 && moving)
    {
      // done with move!
      sprintf(message, "Move complete. New step position = %li; Encoder position = %li\n", (long)get_position(), (long)get_enc_value());
      usb_serial_write(message,strlen(message));
      moving = false;
    }
  }
}

// Parses the serial input
void parse_usb(void)
{
  char buf[100];
  uint32_t count = 0;
  int read = 0;
  int32_t foo;
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
        sprintf(message, "Move Steps mode. Moving from %li by %li steps\n  Current encoder value = %li\n", get_position(), foo, get_enc_value());
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
      // this will be pid control mode
      sprintf(message, "Not implemented yet!\n");
      usb_serial_write(message, strlen(message));
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
          sprintf(message, "Move Steps mode. Moving from %li by %li steps\n  Current encoder value = %li\n", get_position(), foo, get_enc_value());
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
  // which parameter?
  switch(buf[(*i)++])
  {
  case 't':
    // encoder tic count
    sprintf(message, "Encoder Tic Count: %li\n", (long)get_enc_value());
    usb_serial_write(message, strlen(message));
    break;
  case 'm':
    // motor step position
    sprintf(message, "Motor Position (steps): %li\n", (long)get_position());
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
  int read;
  // which parameter?
  switch(buf[(*i)++])
  {
  case 't':
    // encoder tic count
    if(sscanf(buf + *i, " %li%n", (long *)&foo, &read))
    {
      *i += read;
      set_enc_value(foo);
    }
    else
    {
      sprintf(message, "Failed to parse new tic value!\n");
      usb_serial_write(message, strlen(message));
    }
    break;
    
  case 'm':
    // motor step position
    if(sscanf(buf + *i, " %li%n", (long *)&foo, &read))
    {
      *i += read;
      set_position(foo);
    }
    else
    {
      sprintf(message, "Failed to parse new motor position!\n");
      usb_serial_write(message, strlen(message));
    }
    break;
  case 'o':
    // encoder readout frequency
    if(sscanf(buf + *i, " %lu%n", (long *)&foo, &read))
    {
      *i += read;
      show_encoder_time = foo;
    }
    else
    {
      sprintf(message, "Failed to parse new encoder update frequency!\n");
      usb_serial_write(message, strlen(message));
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
      }
      else
      {
        sprintf(message, "Failed to parse new step frequency!\n");
        usb_serial_write(message, strlen(message));
      }
    }
    else
    {
      sprintf(message, "Cannot set step frequency when not in Fixed Step mode or Move Steps mode!\n");
      usb_serial_write(message, strlen(message));
    }
    break;
  default :
    // didn't understand!
    sprintf(message, "I didn't understand which parameter you want to query.\n");
    usb_serial_write(message, strlen(message));
  }
}
