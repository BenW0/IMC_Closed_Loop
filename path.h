/* Path module */
#ifndef __path_h
#define __path_h

#include "imc/protocol/message_structs.h"

// custom path definition structure
typedef struct {
  uint32_t time;
  real target_pos;
  real target_vel;
} custom_path_dp_t;

// path mode enum
typedef enum {
  PATH_STEP,
  PATH_RAMPS_MOVING,    // mode for when we are under trapezoidal motion
  PATH_RAMPS_WAITING,   // mode for when we are waiting for a move to queue so we can start moving
  PATH_CUSTOM,
  PATH_SINES,
  PATH_RAND
} __attribute__ ((packed)) pathmode_t;

void path_set_step_target(int32_t target);

void path_imc(real wait_pos);
void path_ramps_move(volatile msg_queue_move_t *move);

void path_custom_clear(void);
void path_custom_add_elem(const custom_path_dp_t *elem);
void path_custom_start(void);

void path_sines_start(void);
void path_sines_setfreq(float new_base_freq);

void path_rand_start(void);

void path_get_target(volatile real *target_pos, volatile real *target_vel, uint32_t curtime);

#endif