/* Path module

 * This software is (c) 2014 by Ben Weiss and is released under the following license:
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Ben Weiss
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef __path_h
#define __path_h

#include "imc/protocol/message_structs.h"

// custom path definition structure
typedef struct {
  uint32_t time;
  real target_pos;
  real target_vel;
} custom_path_dp_t;

// path mode enum
typedef enum {
  PATH_STEP,
  PATH_RAMPS_MOVING,    // mode for when we are under trapezoidal motion
  PATH_RAMPS_WAITING,   // mode for when we are waiting for a move to queue so we can start moving
  PATH_CUSTOM,
  PATH_SINES,
  PATH_RAND
} __attribute__ ((packed)) pathmode_t;

void path_set_step_target(int32_t target);

void path_imc(real wait_pos);
void path_ramps_move(volatile msg_queue_move_t *move);
uint32_t path_get_ramps_moveid(void);

void path_custom_clear(void);
void path_custom_add_elem(const custom_path_dp_t *elem);
void path_custom_start(void);

void path_sines_start(void);
void path_sines_setfreq(float new_base_freq);

void path_rand_start(void);

void path_get_target(volatile real *target_pos, volatile real *target_vel, uint32_t curtime);

#endif