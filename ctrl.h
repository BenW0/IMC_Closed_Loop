/* Controller module */
#ifndef __ctrl_h
#define __ctrl_h

typedef enum
{
  CTRL_DISABLED,
  CTRL_PID
} ctrl_mode

void ctrl_init(void);

void ctrl_enable(ctrl_mode mode);

void ctrl_set_period(uint32_t us);
uint32_t ctrl_get_period(void);
float ctrl_get_update_time(void);

#endif