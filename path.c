/********************************************************************************
 * Path Module
 * Ben Weiss, University of Washington 2014
 * Purpose: Tracks the desired path of the current move
 * 
 * Controls the definition used for the target position and target
 * velocity fed into the controller. The modes for this module are:
 *   - Step - position target is stepped instantly to the setpoint, velocity target is 0
 *   - RAMPS - uses the RAMPS-standard move packet to create a trapezoidal velocity
 *        profile, integrating to get position target as well.
 *   - Custom - uses a custom path defined by a series of {time, pos_target, vel_target}
 *        frames which define the profile. <time> is in ms since the move began and must
 *        be monotonically-increasing for all elements in the series. Path linearly 
 *        interpolates between the datapoints to generate the pos and vel targets.
 *   - Sines - generates position (tics) and velocity (tics/min) data using summed sinusoids
 *        for system identification purposes.
 * After a RAMPS or Custom move is finished, the path module automatically switches to Step
 * to maintain the final value of the previous move.
 *
 *
 * Source: 
 * 
 * 
 * License: None. This is for internal development only!
 ********************************************************************************/

#include "common.h"
#include <usb_serial.h>
#include <string.h>
#include <stdio.h>

#include "qdenc.h"
#include "spienc.h"
#include "path.h"

// Constants ==========================================================================
#define MAX_CUSTOM_PATH_LENGTH    100     // maximum number of control nodes for a custom path.
#define SINE_COUNT                5       // number of sines for sinusoidal path
const float def_sine_freqs[SINE_COUNT] = {1., 0.865, 0.77777, 0.425, 0.33333};    // rad/tenus
const float sine_shifts[SINE_COUNT] = {0.5, 1.0, -0.2, 0.7, -1.3};            // sine shifts


// Global Variables ====================================================================
//extern uint32_t systick_millis_count;
float sine_freq_base = 1;   // rad/sec
float sine_amp = 20;
uint32_t sine_count = 5;

// Local Variables =====================================================================
static pathmode_t pathmode;        // Type of path we are running.
static int32_t step_target = 0;    // step command target
static msg_queue_move_t curmove;   // current move parameters
static custom_path_dp_t custom_path[MAX_CUSTOM_PATH_LENGTH];
static uint32_t custom_path_curloc = 0;   // upcoming location in the custom_path object
static uint32_t custom_path_length = 0;   // length of current custom_path series.
static uint32_t start_time = 0;    // time we started the current move.
static char message[100];

static float sine_freqs[SINE_COUNT];    // rad/tenus

// tells Path to step instantly to target. This is primarily for debugging, as all real moves
// are ramped moves set with path_set_move.
void path_set_step_target(int32_t target)
{
  step_target = target;
  start_time = get_systick_tenus();
  pathmode = PATH_STEP;
}

void path_set_move(const msg_queue_move_t *move)
{
  // Not implemented yet...
}

void path_custom_clear(void)
{
  custom_path_length = 0;
  custom_path_curloc = 0;
  if(PATH_CUSTOM == pathmode)
    pathmode = PATH_STEP;
}

void path_custom_add_elem(const custom_path_dp_t *elem)
{
  if(custom_path_length < MAX_CUSTOM_PATH_LENGTH - 1)
  {
    custom_path[custom_path_length].target_pos = elem->target_pos;
    custom_path[custom_path_length].target_vel = elem->target_vel;
    custom_path[custom_path_length].time = elem->time;
    custom_path_length++;
  }
}

void path_custom_start(void)
{
  if(custom_path_length > 0)
  {
    pathmode = PATH_CUSTOM;
    start_time = get_systick_tenus();
    custom_path_curloc = 0;
  }
}

void path_sines_start(void)
{
  path_sines_setfreq(sine_freq_base);
  pathmode = PATH_SINES;
  start_time = get_systick_tenus();
}

// sets the frequency of the sine series. new_base_freq is the base frequency in Hz
void path_sines_setfreq(float new_base_freq)
{
  sine_freq_base = new_base_freq;
  for(uint32_t i = 0; i < SINE_COUNT; i++)
  {
    sine_freqs[i] = def_sine_freqs[i] * sine_freq_base * 2 * PI * 0.00001f;   // convert from hz to rad/tenus
  }
}

// curtime is the defined time of this update step, created with a query to get_systic_tenus()
// at the beginning of the control update.
void path_get_target(real *target_pos, real *target_vel, uint32_t curtime)
{
  uint32_t i, elapsed_time;
  int32_t foo;

  // check for counter rollover
  if(curtime < start_time)
    elapsed_time = curtime + UINT32_MAX - start_time;
  else
    elapsed_time = curtime - start_time;


  switch(pathmode)
  {
  case PATH_STEP:
    *target_pos = (real)step_target;
    *target_vel = (real)0.;
    break;
  case PATH_RAMPS :
    // not implemented yet!
    *target_pos = (real)0;
    *target_vel = (real)0;
    break;
  case PATH_CUSTOM :
    {
      custom_path_dp_t *cur, *prev;
      float t;
      // where are we on the current move segment?
      if(custom_path[custom_path_curloc].time < elapsed_time)
      {
        // we have moved beyond the current block and need to load a subsequent one.
        for(i = custom_path_curloc; i < custom_path_length; ++i)
        {
          if(custom_path[i].time > elapsed_time)
          {
            custom_path_curloc = i;
            break;
          }
        }
        if(i == custom_path_length)   // we've exhausted the move sequence
        {
          path_set_step_target((int32_t)custom_path[custom_path_length - 1].target_pos);
          *target_pos = (real)custom_path[custom_path_length - 1].target_pos;
          *target_vel = (real)0;
          break;
        }
      }
      // now the current node in the path is just ahead of us. Let's LERP to it.
      if(custom_path_curloc > 0)    // if it is 0, we'll just return the first datapoint until it's not any more
      {
        cur = &custom_path[custom_path_curloc];
        prev = &custom_path[custom_path_curloc - 1];
        t = (float)(elapsed_time - prev->time) / (float)(cur->time - prev->time);
        *target_pos = (real)lerp(prev->target_pos, cur->target_pos, t);
        *target_vel = (real)lerp(prev->target_vel, cur->target_vel, t);
      }
      else
      {
        *target_pos = (real)custom_path[custom_path_curloc].target_pos;
        *target_vel = (real)custom_path[custom_path_curloc].target_vel;
      }
    }
    break;
  case PATH_SINES :
    *target_pos = 0;
    *target_vel = 0;
    for(uint32_t i = 0; i < sine_count; i++)
    {
      float t = fmodf(elapsed_time, 2*PI / sine_freqs[i]);
      *target_pos += sine_amp * sinf(sine_freqs[i] * t + sine_shifts[i]);
      *target_vel += sine_amp * sine_freqs[i] * cosf(sine_freqs[i] * t + sine_shifts[i]);
    }
    // convert from target_vel being in steps/tenus to steps/min
    *target_vel *= 6000000.f;
    break;
  default :
    // path mode is disabled! Return 0 velocity and the current encoder position
    get_enc_value(&foo);
    *target_pos = (real)foo;
    *target_vel = (real)0.f;
  }
}