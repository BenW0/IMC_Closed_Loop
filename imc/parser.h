#ifndef parser_h
#define parser_h

#include "protocol/message_structs.h"
#include "protocol/constants.h"

#define BUFFER_LENGTH 32
void initialize_i2c(uint8_t);

#define PARSER_NEW_EVENT 0
#define PARSER_EMPTY 1
#define PARSER_ERR 2

typedef struct {
  uint32_t status;
  uint32_t big_packet;
  imc_message_type packet_type;
  uint32_t remaining;
  uint8_t* head;
  union {
    msg_initialize_t init;
    msg_queue_move_t move;    
    msg_get_param_t get_param;
    msg_set_param_t set_param;
    uint8_t pad[sizeof(msg_queue_move_t) + 1];
  } packet;
} parser_state_t;

typedef union {
  rsp_initialize_t init;
  rsp_status_t status;
  rsp_get_param_t param;
} generic_response;

extern volatile parser_state_t parser;

void initialize_parser(void);
void feed_data(uint8_t);

extern generic_response response;
// Response type, and number of bytes of response to write
void send_response(imc_response_type,uint32_t);

extern volatile uint8_t* txHead;
extern volatile uint32_t txRemaining;
extern volatile uint32_t irqcount;
#endif
