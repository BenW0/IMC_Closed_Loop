/*
 * This software is (c) 2014 by Ben Weiss and is released under the following license:
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Matthew D Sorensen and Ben Weiss
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
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

