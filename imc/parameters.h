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

void reset_parameters(void);
void handle_get_parameter(volatile msg_get_param_t*,rsp_get_param_t*);
void handle_set_parameter(volatile msg_set_param_t*);

#endif
