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

#include "imc/stepper.h"
#include "imc/utils.h"
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
extern float max_ctrl_vel;
extern float enc_tics_per_step;
float sine_freq_base = 1;   // rad/sec
float sine_amp = 20;
float rand_scale = 1.f;
uint32_t sine_count = 5;

// Local Variables =====================================================================
static pathmode_t pathmode;        // Type of path we are running.
static int32_t step_target = 0;    // step command target
static custom_path_dp_t custom_path[MAX_CUSTOM_PATH_LENGTH];
static uint32_t custom_path_curloc = 0;   // upcoming location in the custom_path object
static uint32_t custom_path_length = 0;   // length of current custom_path series.
static uint32_t start_time = 0;    // time we started the current move.
static real last_target_pos = 0;

static struct {
  real accel;
  real v_final;
  real v_init;
  real v_nom;
  real x_total;

  real start_pos;   // position when we started the move. This is defined as "x = 0"
  real dir;         // 1 => forward; -1 => backwards
  bool short_move;  // true if we never reach v_nom during the move.

  real t1;     // time (tenus) when we reach the nominal rate, if we get there, or the time we switch from accel to decel if we don't.
  real t2;     // time (tenus) when we start decelerating
  real t3;     // time (tenus) when move is complete

  real x1;     // position at time t1.
  real x2;     // position at time t2.

  real vp;      // peak velocity in the event a move doesn't reach v_nom.
} rmove;
static real ramps_endpos = 0;

static float sine_freqs[SINE_COUNT];    // rad/tenus

// Local functions ========================================================
void get_targets_ramps(volatile real *target_pos, volatile real *target_vel, uint32_t t);

// tells Path to step instantly to target. This is primarily for debugging, as all real moves
// are ramped moves set with path_set_move.
void path_set_step_target(int32_t target)
{
  step_target = target;
  ramps_endpos = target;    // in case we do a ramps move next...
  start_time = get_systick_tenus();
  pathmode = PATH_STEP;
}

void path_imc(real wait_pos)
{
  ramps_endpos = wait_pos;
  start_time = get_systick_tenus();
  pathmode = PATH_RAMPS_WAITING;
}

// Implements a trapezoidal velocity profile move, as specified in the same way as packets from 
// the original IMC interface. References to Eqn are links to the equations listed in my notes,
// dated 5/31/2014
void path_ramps_move(volatile msg_queue_move_t *move)
{
  real ratio;

  // put us in waiting mode, just in case the stepper interrupt runs while we're processing this section.
  if(PATH_RAMPS_MOVING == pathmode)
  {
    ramps_endpos = rmove.start_pos + rmove.x_total * rmove.dir;
    pathmode = PATH_RAMPS_WAITING;
  }

  start_time = get_systick_tenus();   //||\\ Change this later?

  // set up the rmove structure. We will convert everything here into tics and seconds, and scale to
  // adjust velocities and accelerations depending on how far we actually need to move (for synchronized
  // motion in more than one axis)
  ratio = (real)fabsf(move->length) / (real)move->total_length;
  rmove.accel = ratio * (real)move->acceleration * enc_tics_per_step * MIN_PER_TENUS_F * MIN_PER_TENUS_F;   // (steps/min^2) * (tics/step) * (min/tenus)^2
  rmove.v_init = ratio * (real)move->initial_rate * enc_tics_per_step * MIN_PER_TENUS_F;              // (steps/min) * (tics/step) * (min/tenus)
  rmove.v_final = ratio * (real)move->final_rate * enc_tics_per_step * MIN_PER_TENUS_F;
  rmove.v_nom = ratio * (real)move->nominal_rate * enc_tics_per_step * MIN_PER_TENUS_F;
  rmove.x_total = fabsf((real)move->length) * enc_tics_per_step;
  rmove.dir = move->length >= 0 ? 1.f : -1.f;

  rmove.start_pos = (real)ramps_endpos;      // position defined as "x = 0"

  // compute t1 and t2
  rmove.t1 = (rmove.v_nom - rmove.v_init) / rmove.accel;       // Eqn (2). Units: tenus.
  rmove.x1 = rmove.t1 * (rmove.v_init + 0.5f * rmove.accel * rmove.t1);    // Eqn (1)
  rmove.x2 = rmove.x_total - (rmove.v_nom * rmove.v_nom - rmove.v_final * rmove.v_final) / (2.f * rmove.accel);   // Eqn (9). Units: tics
  rmove.t2 = rmove.t1 + (rmove.x2 - rmove.x1) / rmove.v_nom;    // Eqn (5). Units: tenus
  rmove.t3 = rmove.t2 + (rmove.v_nom - rmove.v_final) / rmove.accel;

  // is this a short move?
  rmove.short_move = rmove.t1 > rmove.t2;
  if(rmove.short_move)
  {
    rmove.vp = sqrtf(0.5f * (rmove.v_init * rmove.v_init + rmove.v_final * rmove.v_final + 2 * rmove.accel * rmove.x_total));
    rmove.x1 = (rmove.vp * rmove.vp - rmove.v_init * rmove.v_init) / (2.f * rmove.accel);
    rmove.t1 = (rmove.vp - rmove.v_init) / rmove.accel;
    rmove.t3 = (2 * rmove.vp - rmove.v_init - rmove.v_final) / rmove.accel;
  }

  /*serial_printf("accel = %g, v_init = %g, v_final = %g\n\
v_nom = %g, x_total = %g, dir = %g\n\
start_pos = %g t1 = %g, t2 = %g, t3 = %g\n\
x1 = %g, x2 = %g, short_move = %i, vp = %g\n",
                rmove.accel, rmove.v_init, rmove.v_final,
                rmove.v_nom, rmove.x_total, rmove.dir, 
                rmove.start_pos, rmove.t1, rmove.t2, rmove.t3,
                rmove.x1, rmove.x2, rmove.short_move, rmove.vp);*/

  pathmode = PATH_RAMPS_MOVING;
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

void path_rand_start(void)
{
  pathmode = PATH_RAND;
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
void path_get_target(volatile real *target_pos, volatile real *target_vel, uint32_t curtime)
{
  static uint32_t last_time = 0;
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
  case PATH_RAMPS_WAITING:
    // waiting for a new move packet (buffer was empty last time we tried)
    //||\\ TODO check for move to dequeue.
    *target_pos = (real)ramps_endpos;
    *target_vel = (real)0.;
    break;
  case PATH_RAMPS_MOVING :
    get_targets_ramps(target_pos, target_vel, elapsed_time);
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
    *target_vel *= TENUS_PER_MIN_F;
    break;
  case PATH_RAND :
    {
      // move by at most max_ctrl_vel tics/minute.
      *target_vel = 0;      // I don't want to think about how to set this right now...
      if(elapsed_time - last_time < 50000U)   // if it's been < 50ms
        *target_pos = last_target_pos + rand_scale * 2.f * ((real)rand_uint32() - (real)UINT_FAST32_MAX * 0.5f) / (real)UINT_FAST32_MAX * (real)max_ctrl_vel / TENUS_PER_MIN_F * (real)(elapsed_time - last_time);
      else
        *target_pos = last_target_pos;
    }
    break;
  default :
    // path mode is disabled! Return 0 velocity and the current encoder position
    get_enc_value(&foo);
    *target_pos = (real)foo;
    *target_vel = (real)0.f;
  }

  last_time = elapsed_time;
  last_target_pos = *target_pos;
}

// gets the targets when in a RAMPS move, using the contents of the rmove structure.
void get_targets_ramps(volatile real *target_pos, volatile real *target_vel, uint32_t t)
{
  // check for stepper module errors (IMC end stop hit, etc.)
  if(st.state != STATE_EXECUTE)
  {
    // Something went horribly wrong!
    serial_printf("'Unexpected stepper state change in get_targets_ramps!\n");
    // just station-keep here.
    *target_pos = last_target_pos;
    *target_vel = 0.f;
    pathmode = PATH_RAMPS_WAITING;
    ramps_endpos = last_target_pos;
    return;
  }

  // short move?
  if(rmove.short_move)   // we never reach the flat part of the trapezoid. This move has a trianglular velocity profile
  {
    if(t < rmove.t1)
    {
      *target_pos = t * (rmove.v_init + 0.5f * rmove.accel * t);    // Eqn (11)
      *target_vel = rmove.v_init + rmove.accel * t;                                               // Eqn (12)
    }
    else if(t < rmove.t3)   // there is no t2.
    {
      *target_pos = rmove.x1 + (t - rmove.t1) * (rmove.vp - 0.5f * rmove.accel * (t - rmove.t1));
      *target_vel = rmove.vp - rmove.accel * (t - rmove.t1);
    }
    else    // move finished
    {
      //||\\TODO Dequeue a new move and start that one, if available. For now, we'll just go to waiting mode
      pathmode = PATH_RAMPS_WAITING;
      ramps_endpos = rmove.start_pos + rmove.dir * rmove.x_total;
      *target_pos = rmove.x_total;
      *target_vel = rmove.v_final;
      float_sync();   // tell the stepper module to float the sync line, signaling we're finished with the move.
    }
  }
  else    // normal move
  {
    // figure out which chunk we are in
    if(t < rmove.t1)    // first region, accelerating
    {
      *target_pos = t * (rmove.v_init + 0.5f * t * rmove.accel);    // Eqn (0.5)
      *target_vel = rmove.v_init + rmove.accel * t;                                               // Eqn (0)
    }
    else if(t < rmove.t2) // flat region
    {
      *target_pos = (rmove.x1 + rmove.v_nom * (t - rmove.t1));      // Eqn (4)
      *target_vel = rmove.v_nom;    // Eqn (3)
    }
    else if(t < rmove.t3) // descelerating region
    {
      *target_pos = (rmove.x2 + (t - rmove.t2) * (rmove.v_nom  - 0.5f * (t - rmove.t2) * rmove.accel)); // Eqn (7)
      *target_vel = rmove.v_final + rmove.accel * (rmove.t3 - t);
    }
    else    // move finished
    {
      pathmode = PATH_RAMPS_WAITING;
      ramps_endpos = rmove.start_pos + rmove.dir * rmove.x_total;
      *target_pos = rmove.x_total;
      *target_vel = rmove.v_final;
      float_sync();   // tell the stepper module to float the sync line, signaling we're finished with the move.
    }
  }
  *target_pos = *target_pos * rmove.dir + rmove.start_pos;
  *target_vel *= TENUS_PER_MIN_F * rmove.dir;   // get velocity back into tics/min.
}