/* Controller module */
#ifndef __ctrl_h
#define __ctrl_h

typedef enum
{
  CTRL_DISABLED,    // module disabled.
  CTRL_UNITY,       // Controller is G(z) = 1
  CTRL_PID,          // PID control mode
  CTRL_BANG,         // bang-bang control mode
  CTRL_DARMA,        // DARMA control mode
  CTRL_COMP           // compensating filter controller
} ctrl_mode;


#define FILTER_MAX_SIZE 8      // maximum number of terms in any controller that uses a filter (darma/comp). Ring buffer...needs to be a power of 2.

void init_ctrl(void);

void ctrl_enable(ctrl_mode mode);

void ctrl_set_period(uint32_t us);
uint32_t ctrl_get_period(void);
float ctrl_get_update_time(void);
void output_history(void);

#endif