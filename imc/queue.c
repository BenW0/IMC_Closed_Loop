#include "protocol/message_structs.h"
#include "queue.h"
#include "utils.h"
#include <string.h>

static msg_queue_move_t motion_queue[MOTION_QUEUE_LENGTH];
static volatile uint32_t queue_head;
static volatile uint32_t queue_size;

void initialize_motion_queue(void){
  memset(motion_queue, 0, sizeof(msg_queue_move_t) * MOTION_QUEUE_LENGTH);
  queue_size = queue_head = 0;
}

int enqueue_block(volatile msg_queue_move_t* src){
  if(queue_size == MOTION_QUEUE_LENGTH)
    return -1;
  uint32_t offset = (queue_head + queue_size) & MOTION_QUEUE_MASK;
  vmemcpy(&(motion_queue[offset]), src, sizeof(msg_queue_move_t));
  queue_size++;
  return MOTION_QUEUE_LENGTH - queue_size;
}

msg_queue_move_t* dequeue_block(void){
  msg_queue_move_t* ret;
  if(queue_size == 0)
    return NULL;
  queue_size--;
  ret = &(motion_queue[queue_head]);
  queue_head = (queue_head + 1) & MOTION_QUEUE_MASK;
  return ret;
}

uint32_t queue_length(void){
  return queue_size;
}

