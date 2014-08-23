/*

 * Code credit for random number generation: http://stackoverflow.com/questions/1167253/implementation-of-rand
 
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

#ifndef stepper_hooks_h
#define stepper_hooks_h

// initialize the stepper hooks
void init_stepper_hooks(void);
// Get/Set steps to go - limit move length by a number of steps. Positive unless disabled (=-1).
void set_steps_to_go(int32_t steps);
int32_t get_steps_to_go(void);
// set step rate
void set_step_events_per_minute_ctrl(uint32_t); 
uint32_t get_step_events_per_minute(void);
// start motion (like execute_move(), but does not dequeue a move since we're not in IMC mode.
void start_moving(void);
#endif
