#include "parameters.h"
#include "hardware.h"
#include "stepper.h"
#include "peripheral.h"
#include "protocol/constants.h"
#include "protocol/message_structs.h"

parameters_t parameters;

// hooks to allow the user of the imc module to extend the parameters interface.
static void (*param_reset_hook)(void) = NULL;
static void (*param_get_hook)(volatile msg_get_param_t* ,rsp_get_param_t* ) = NULL;
static void (*param_set_hook)(volatile msg_set_param_t* ) = NULL;

void set_param_hooks(void (*reset_hook)(void), void (*get_hook)(volatile msg_get_param_t* ,rsp_get_param_t* ),
                void (*set_hook)(volatile msg_set_param_t*))
{
  param_reset_hook = reset_hook;
  param_get_hook = get_hook;
  param_set_hook = set_hook;
}

void reset_parameters(void){
  parameters.error_low = 0;
  parameters.error_high = 0;
  parameters.homing = ENABLE_MIN; // Don't flip, home low, no software max, no invert
  parameters.min_pos = 0;
  parameters.max_pos = 0;

  parameters.home_pos = 0;
  parameters.homing_feedrate = 60000; // Arbitrary, but reasonable
  parameters.motor_on = 0;
  parameters.motor_timeout = 1<<30; // Big number is big
  // Position lives in the stepper state, as it is volatile
  parameters.slowdown = 0;
  parameters.sync_error = 0;
  parameters.last_home = 0;

  if(param_reset_hook)
    param_reset_hook();
}

static uint32_t const_to_mask(imc_axis_parameter c){
  switch(c){
  case IMC_PARAM_FLIP_AXIS:
    return FLIP_AXIS;
  case IMC_PARAM_HOME_DIR:
    return HOME_DIR;
  case IMC_PARAM_MIN_SOFTWARE_ENDSTOPS:
    return MIN_SOFTWARE;
  case IMC_PARAM_MAX_SOFTWARE_ENDSTOPS:
    return MAX_SOFTWARE;
  case IMC_PARAM_MIN_LIMIT_EN:
    return ENABLE_MIN;
  case IMC_PARAM_MAX_LIMIT_EN:
    return ENABLE_MAX;
  case IMC_PARAM_MIN_LIMIT_INV:
    return INVERT_MIN;
  case IMC_PARAM_MAX_LIMIT_INV:
    return INVERT_MAX;
  default:
    return 0;
  }
}

static uint32_t limit_state(uint32_t dir){
  uint32_t bit = dir ? MAX_LIMIT_BIT : MIN_LIMIT_BIT;
  uint32_t invert = (parameters.homing & (dir ? INVERT_MAX : INVERT_MIN)) ? bit : 0;
  return ((CONTROL_PORT(DIR) ^ invert) & bit) ? 1 : 0;
}



void handle_get_parameter(volatile msg_get_param_t* msg,rsp_get_param_t* rsp){
  switch(msg->param_id){
  case IMC_PARAM_ERROR_INFO1:
    rsp->value = parameters.error_low;
    break;
  case IMC_PARAM_ERROR_INFO2:
    rsp->value = parameters.error_high;
    break;
  case IMC_PARAM_FLIP_AXIS:
  case IMC_PARAM_HOME_DIR:
  case IMC_PARAM_MIN_SOFTWARE_ENDSTOPS:
  case IMC_PARAM_MAX_SOFTWARE_ENDSTOPS:
  case IMC_PARAM_MIN_LIMIT_EN:
  case IMC_PARAM_MAX_LIMIT_EN:
  case IMC_PARAM_MIN_LIMIT_INV:
  case IMC_PARAM_MAX_LIMIT_INV:
    rsp->value = const_to_mask(msg->param_id) & parameters.homing ? 1 : 0;
    break;
  case IMC_PARAM_MIN_POS:
    rsp->value = parameters.min_pos;
    break;
  case IMC_PARAM_MAX_POS:
    rsp->value = parameters.max_pos;
    break;
  case IMC_PARAM_HOME_POS:
    rsp->value = parameters.home_pos;
    break;
  case IMC_PARAM_HOMING_FEEDRATE:
    rsp->value = parameters.homing_feedrate;
    break;
  case IMC_PARAM_MOTOR_ON:
    rsp->value = parameters.motor_on;
    break;
  case IMC_PARAM_MOTOR_IDLE_TIMEOUT:
    rsp->value = parameters.motor_timeout;
    break;
  case IMC_PARAM_LOCATION:
    rsp->value = get_motor_position();
    break;
  case IMC_PARAM_SLOWDOWN:
    rsp->value = parameters.slowdown;
    break;
  case IMC_PARAM_SYNC_ERROR:
    rsp->value = parameters.sync_error;
    break;
  case IMC_PARAM_LAST_HOME:
    rsp->value = parameters.last_home;
    break;
  case IMC_PARAM_MIN_LIMIT_STATE:
    rsp->value = limit_state(0);
    break;
  case IMC_PARAM_MAX_LIMIT_STATE:
    rsp->value = limit_state(1);
    break;
  default:
    ;
  }

  if(param_get_hook)
    param_get_hook(msg, rsp);
}

void handle_set_parameter(volatile msg_set_param_t* msg){
  // Can't set error info or sync_error
  // Pullups and motor on/off are special, as we actually have to do io
  uint32_t mask;
  uint32_t axis = 0;
  switch(msg->param_id){
  case IMC_PARAM_MAX_LIMIT_INV:
  case IMC_PARAM_MAX_LIMIT_EN:
    axis = 1;
  case IMC_PARAM_MIN_LIMIT_EN:
  case IMC_PARAM_MIN_LIMIT_INV:
  case IMC_PARAM_FLIP_AXIS:
  case IMC_PARAM_HOME_DIR:
  case IMC_PARAM_MIN_SOFTWARE_ENDSTOPS:
  case IMC_PARAM_MAX_SOFTWARE_ENDSTOPS:
    mask = const_to_mask(msg->param_id);
    parameters.homing = ((~mask) & parameters.homing) | (msg->param_value ? mask : 0);
    // This is called a few extra times, but should be idempotent
    configure_limit_gpio(axis, PRESERVE_PULLUP, parameters.homing);
    break;
  case IMC_PARAM_MIN_POS: parameters.min_pos = msg->param_value; break;
  case IMC_PARAM_MAX_POS: parameters.max_pos = msg->param_value; break;

  case IMC_PARAM_HOMING_FEEDRATE: 
    {
      uint32_t minimum = msg->param_value;
      minimum = minimum < MINIMUM_STEPS_PER_MINUTE ? MINIMUM_STEPS_PER_MINUTE : minimum;
      parameters.homing_feedrate = minimum;
    }
    break;
  case IMC_PARAM_HOME_POS: parameters.home_pos = msg->param_value; break;
  case IMC_PARAM_MOTOR_IDLE_TIMEOUT: parameters.motor_timeout = msg->param_value; break;
  case IMC_PARAM_SLOWDOWN: parameters.slowdown = msg->param_value; break;
  case IMC_PARAM_MIN_LIMIT_PULLUP:
    configure_limit_gpio(0, msg->param_value, parameters.homing);
    break;
  case IMC_PARAM_MAX_LIMIT_PULLUP:
    configure_limit_gpio(1, msg->param_value, parameters.homing);
    break;
  case IMC_PARAM_MOTOR_ON:
    // Do some IO to turn the motor on or off
    enable_stepper();
    parameters.motor_on = msg->param_value; 

    if(parameters.motor_on){
      enable_stepper();
    }else{
      disable_stepper();
    }
    break;
  case IMC_PARAM_LOCATION:
    // This may have a signed/unsigned issue. todo: figure that out
    set_motor_position(msg->param_value);
    break;
  case IMC_PARAM_MICROSTEPPING:
    set_microstepping(msg->param_value);
    break;
  default:
    break;
  }
  
  if(param_set_hook)
    param_set_hook(msg);
}
