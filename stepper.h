#ifndef stepper_h
#define stepper_h

// Motion parameters - see grbl for documentation
#define ACCELERATION_TICKS_PER_SECOND 50L
#define MINIMUM_STEPS_PER_MINUTE 800 // (steps/min) - Integer value only

// The motion state machine is both simpler than grbl (less interactive)
// and more complex, given our sync mechanism.
typedef enum {
  STATE_IDLE,    // We have moves in the queue but no sync trigger yet, or have an empty queue
  STATE_EXECUTE, // We're currently executing a move
  STATE_SYNC,   // We've finished executing a move, and are waiting for sync to go low
  STATE_ERROR
} execution_state_t;

typedef struct {
  // More or less pulled straight from grbl!
  // Used by the bresenham line algorithm
  int32_t counter;        // Counter variables for the bresenham line tracer
  uint32_t event_count;
  uint32_t step_events_completed;  // The number of step events left in current motion
  uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
  uint32_t min_safe_rate;  // Minimum safe rate for full deceleration rate reduction step. Otherwise halves step_rate.

  // The following fields are taken from the grbl sys data structure
  int32_t position; // Current position, in number of steps
  execution_state_t state; // motion state parameters
  
  // cycles_per_step_event to use next time we update
  uint32_t new_cycles_per_step_event;
 
} stepper_state_t;

extern volatile stepper_state_t st;
// Execute the next move in the queue
void execute_move(void);
// Immediately kill all motion, probably killing position
void stop_motion(void);
// Reset all state, but don't touch IO
void initialize_stepper_state(void);
// Power up the stepper motor
void enable_stepper(void);
// Power down the stepper motor
void disable_stepper(void);
// Getters and setters for current position - used for homing and reporting positions
int32_t get_position(void);
void set_position(uint32_t);
// Get/Set steps to go - limit move length by a number of steps. Positive unless disabled (=-1).
void set_steps_to_go(int32_t steps);
int32_t get_steps_to_go(void);
// Trigger a pulse on the step pin
void trigger_pulse(void);
// set step rate
void set_step_events_per_minute(uint32_t); 
uint32_t get_step_events_per_minute(void);
// set direction
void set_direction(bool);
bool get_direction(void);
#endif
