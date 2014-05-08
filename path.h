/* Path module */
#ifndef __path_h
#define __path_h

typedef struct {
  int32_t length;
  uint32_t total_length;
  uint32_t initial_rate;
  uint32_t nominal_rate;
  uint32_t final_rate;
  uint32_t acceleration;
  uint32_t stop_accelerating;
  uint32_t start_decelerating;
} __attribute__ ((packed)) msg_queue_move_t;

// custom path definition structure
typedef struct {
  uint32_t time;
  real target_pos;
  real target_vel;
} custom_path_dp_t;

// path mode enum
typedef enum {
  PATH_STEP,
  PATH_RAMPS,
  PATH_CUSTOM,
  PATH_SINES
} __attribute__ ((packed)) pathmode_t;

void path_set_step_target(int32_t target);
void path_set_move(const msg_queue_move_t *move);

void path_custom_clear(void);
void path_custom_add_elem(const custom_path_dp_t *elem);
void path_custom_start(void);

void path_sines_start(void);
void path_sines_setfreq(float new_base_freq);

void path_get_target(real *target_pos, real *target_vel, uint32_t curtime);

#endif