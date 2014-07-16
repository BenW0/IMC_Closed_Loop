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