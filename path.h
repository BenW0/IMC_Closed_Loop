/* Path module */
#ifndef __path_h
#define __path_h

typedef struct { // Assume we don't have to pad this on 32-bit systems
  int32_t length;
  uint32_t total_length;
  uint32_t initial_rate;
  uint32_t nominal_rate;
  uint32_t final_rate;
  uint32_t acceleration;
  uint32_t stop_accelerating;
  uint32_t start_decelerating;
} __attribute__ ((packed)) msg_queue_move_t;

void path_set_step_target(int32_t target);
void path_set_move(const msg_queue_move_t *move);
void path_get_target(float *target_pos, float *target_vel);

#endif