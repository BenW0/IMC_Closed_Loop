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
// set direction
void set_direction(bool);
bool get_direction(void);
// start motion (like execute_move(), but does not dequeue a move since we're not in IMC mode.
void start_moving(void);
#endif
