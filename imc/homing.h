#ifndef homing_h
#define homing_h

#include <stdbool.h>

void set_homing_hooks(bool (*start_hook)(), void (*end_hook)());
void enter_homing_routine(void);

#endif
