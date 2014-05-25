#ifndef queue_h
#define queue_h
#include <stdint.h>
#include "protocol/message_structs.h"

#define MOTION_QUEUE_LENGTH 16
#define MOTION_QUEUE_MASK 0xF

void initialize_motion_queue(void);

int enqueue_block(volatile msg_queue_move_t*);

msg_queue_move_t*  dequeue_block(void);

uint32_t queue_length(void);

#endif
