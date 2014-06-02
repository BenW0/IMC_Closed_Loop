#ifndef stepper_h
#include <stdint.h>
#include <stdbool.h>
#include "protocol/message_structs.h"
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
  // Used by the trapezoid generator
  uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
  uint32_t trapezoid_tick_cycle_counter; // The cycles since last trapezoid_tick. Used to generate ticks at a steady
                                         // pace without allocating a separate timer
  uint32_t trapezoid_adjusted_rate;      // The current rate of step_events according to the trapezoid generator
  uint32_t min_safe_rate;  // Minimum safe rate for full deceleration rate reduction step. Otherwise halves step_rate.

  // The following fields are taken from the grbl sys data structure
  int32_t position; // Current position, in number of steps
  execution_state_t state; // motion state parameters
 
} stepper_state_t;

extern volatile stepper_state_t st;
extern volatile msg_queue_move_t* current_block;    // made this public so path.c can see it.

// Execute the next move in the queue
void execute_move(void);
// Immediately kill all motion, probably killing position
void stop_motion(void);
// Reset all state, but don't touch IO
void initialize_stepper_state(void);
// routines for configuring hooks
void set_step_hook(bool (*stephook)(void));
void set_execute_hook(bool (*exechook)(void));
// sets the step frequency
uint32_t config_step_timer(uint32_t cycles);
// direction commands
void set_direction(bool backwards);
bool get_direction(void);
// Power up the stepper motor
void enable_stepper(void);
// Power down the stepper motor
void disable_stepper(void);
// Getters and setters for current position - used for homing and reporting positions
int32_t get_motor_position(void);
void set_motor_position(uint32_t);
// Trigger a pulse on the step pin
void trigger_pulse(void);
#endif
