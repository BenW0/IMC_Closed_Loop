/* Controller module 

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
#ifndef __ctrl_h
#define __ctrl_h

typedef enum
{
  CTRL_DISABLED,    // module disabled.
  CTRL_UNITY,       // Controller is G(z) = 1
  CTRL_PID,          // PID control mode
  CTRL_BANG,         // bang-bang control mode
  CTRL_DARMA,        // DARMA control mode
  CTRL_COMP,         // compensating filter controller
} ctrl_mode;


#define FILTER_MAX_SIZE 8      // maximum number of terms in any controller that uses a filter (darma/comp). Ring buffer...needs to be a power of 2.

void init_ctrl(void);

void ctrl_enable(ctrl_mode mode);
ctrl_mode ctrl_get_mode(void);

void ctrl_set_period(uint32_t us);
uint32_t ctrl_get_period(void);
float ctrl_get_update_time(void);
void output_history(void);

#endif