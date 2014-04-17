/********************************************************************************
 * Path Module
 * Ben Weiss, University of Washington 2014
 * Purpose: Tracks the desired path of the current move
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

#include "path.h"

// Global Variables ====================================================================
extern uint32_t systick_millis_count;

// Local Variables =====================================================================
static bool step_mode = true;      // True if the last set command we received was a step (instead of a move)
static int32_t step_target = 0;    // step command target
static msg_queue_move_t curmove;   // current move parameters
static uint32_t start_time = 0;    // time we started the current move.

// tells Path to step instantly to target. This is primarily for debugging, as all real moves
// are ramped moves set with path_set_move.
void path_set_step_target(int32_t target)
{
  step_target = target;
  start_time = systick_millis_count;
  step_mode = true;
}

void path_set_move(const msg_queue_move_t *move)
{
  // Not implemented yet...
}

void path_get_target(float *target_pos, float *target_vel)
{
  if(step_mode)
  {
    *target_pos = step_target;
    *target_vel = 0.;
  }
  else
  {
    // move mode -- not implemented yet.
  }
}