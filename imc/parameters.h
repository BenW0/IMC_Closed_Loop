/*
 * This software is (c) 2014 by Ben Weiss and is released under the following license:
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Matthew D Sorensen and Ben Weiss
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
#ifndef parameter_h
#define parameter_h

#include "protocol/constants.h"
#include "protocol/message_structs.h"
#include <stdint.h>

#define FLIP_AXIS (1<<0)
#define HOME_DIR  (1<<1)
#define MIN_SOFTWARE (1<<2)
#define MAX_SOFTWARE (1<<3)
#define ENABLE_MIN (1<<4)
#define ENABLE_MAX (1<<5)
#define INVERT_MIN (1<<6)
#define INVERT_MAX (1<<7)

#define PRESERVE_PULLUP (IMC_PULLUP + IMC_PULLDOWN)

typedef struct {
  uint32_t error_low;
  uint32_t error_high;
  uint32_t homing; // Bit mask with fields defined by the constants at top of file
  uint32_t min_pos;
  uint32_t max_pos;
  uint32_t home_pos;
  uint32_t homing_feedrate;
  uint32_t motor_on;
  uint32_t motor_timeout;
  // Position lives in the stepper state
  uint32_t slowdown;
  uint32_t sync_error;
  int32_t  last_home;
} parameters_t;

extern parameters_t parameters;

// parameter hooks allow the host project to extend the list of parameters without further code modifications.
void set_param_hooks(void (*reset_hook)(void), void (*get_hook)(volatile msg_get_param_t* ,rsp_get_param_t* ),
                void (*set_hook)(volatile msg_set_param_t*));

void reset_parameters(void);
void handle_get_parameter(volatile msg_get_param_t*,rsp_get_param_t*);
void handle_set_parameter(volatile msg_set_param_t*);

#endif
