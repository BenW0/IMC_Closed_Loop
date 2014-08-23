/*
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
#include "common.h"

#include "param_hooks.h"
#include "qdenc.h"
#include "spienc.h"
#include "imc/parameters.h"
#include "path.h"
#include "imc/stepper.h"


// Global Variables ===========================================================
extern float enc_tics_per_step;
extern float steps_per_enc_tic;

void param_reset_hook(void);
void param_get_hook(volatile msg_get_param_t* ,rsp_get_param_t* );
void param_set_hook(volatile msg_set_param_t* );


// Sets up the parameters hooks so that we can be notified of parameter sets/gets.
void init_param_hooks(void)
{
  set_param_hooks(param_reset_hook, param_get_hook, param_set_hook);
}


void param_reset_hook(void)
{
  // not sure we actually need to do anything here at this point...
}

void param_get_hook(volatile msg_get_param_t *msg ,rsp_get_param_t *rsp )
{
  int32_t foo;
  switch(msg->param_id)
  {
  case IMC_PARAM_LOCATION :// override requests for position with the encoder-driven position
    get_enc_value(&foo);
    rsp->value = (int32_t)((float)foo * steps_per_enc_tic);
    break;
  }
}

void param_set_hook(volatile msg_set_param_t *msg )
{
  int32_t foo;
  switch(msg->param_id)
  {
  case IMC_PARAM_LOCATION :// also set the position of the encoder.
    set_enc_value((int32_t)((float)msg->param_value * enc_tics_per_step));
    path_imc(get_motor_position() * enc_tics_per_step);   // keep the controller from moving us back to where we were.
    break;
  }
}