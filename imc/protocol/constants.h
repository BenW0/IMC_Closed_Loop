/* 
This file contains definitions for all constants in the specification of the IMCC protocol.

(C) 2014, Ben Weiss & Matthew Sorensen
 */
#ifndef imc_protocol_constants_h
#define imc_protocol_constants_h

// constants in this enum need to match indices into the imc_message_length[] and imc_resp_length[] arrays in message_structs.h
typedef enum {
  IMC_MSG_INITIALIZE = 1,
  IMC_MSG_STATUS = 2,
  IMC_MSG_HOME = 3,
  IMC_MSG_QUEUEMOVE = 4,
  IMC_MSG_GETPARAM = 5,
  IMC_MSG_SETPARAM = 6,
  IMC_MSG_QUICKSTOP = 7
  // FUTURE: IMC_MSG_BABYSTEP
} __attribute__ ((packed)) imc_message_type;


typedef enum   {
  IMC_RSP_COMM_ERROR = 0, // 0 is special - since it's easy to get 0's by accident, they are defined as a special error.
  IMC_RSP_OK,
  IMC_RSP_UNKNOWN,
  IMC_RSP_ERROR,
  IMC_RSP_QUEUEFULL
} __attribute__ ((packed)) imc_response_type;

typedef enum {
  IMC_PARAM_ERROR_INFO1, // Read only
  IMC_PARAM_ERROR_INFO2, // Read only
  IMC_PARAM_FLIP_AXIS,
  IMC_PARAM_HOME_DIR,
  IMC_PARAM_MIN_SOFTWARE_ENDSTOPS,
  IMC_PARAM_MAX_SOFTWARE_ENDSTOPS,
  IMC_PARAM_MIN_LIMIT_EN,
  IMC_PARAM_MIN_LIMIT_INV,
  IMC_PARAM_MIN_LIMIT_PULLUP, // Write only
  IMC_PARAM_MAX_LIMIT_EN,
  IMC_PARAM_MAX_LIMIT_INV, 
  IMC_PARAM_MAX_LIMIT_PULLUP, // Write only
  IMC_PARAM_MIN_POS,
  IMC_PARAM_MAX_POS,
  IMC_PARAM_HOME_POS,
  IMC_PARAM_HOMING_FEEDRATE,
  IMC_PARAM_MOTOR_ON,
  IMC_PARAM_MOTOR_IDLE_TIMEOUT,
  IMC_PARAM_SLOWDOWN,
  IMC_PARAM_LOCATION,
  IMC_PARAM_SYNC_ERROR, // Read only
  IMC_PARAM_LAST_HOME,  // Read only
  IMC_PARAM_MICROSTEPPING, // Write only
  IMC_PARAM_MIN_LIMIT_STATE, // Read only
  IMC_PARAM_MAX_LIMIT_STATE  // Read only
} __attribute__ ((packed)) imc_axis_parameter;

typedef enum {
  IMC_NO_PULL,
  IMC_PULLUP,
  IMC_PULLDOWN,
} __attribute__ ((packed)) imc_pullup_values;

typedef enum {
  IMC_ERR_NONE,
  IMC_ERR_CONTROL,
  IMC_ERR_ELECTRICAL,
  IMC_ERR_MECHANICAL,
  IMC_ERR_TIMEOUT
} __attribute__ ((packed)) imc_axis_error;

#endif
