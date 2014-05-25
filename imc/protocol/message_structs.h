#ifndef message_structs_h
#define message_structs_h

#include <stdint.h>
#include "constants.h"

// Critical! All packets above this size MUST be split into two transmissions
#define PROTOCOL_MAX_TRANSMIT_SIZE 32

////////////////////// Message Packet Structures /////////////////////////////////
// All
typedef struct {
  uint16_t host_revision;
  uint8_t  reserved[6];
} __attribute__ ((packed)) msg_initialize_t;

//typedef struct {
//  ;  // no fields in message
//} __attribute__ ((packed)) msg_status_t;

//typedef struct {
//  ;  // no fields in message
//} __attribute__ ((packed)) msg_home_t;

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

typedef struct {
  uint8_t param_id;
} __attribute__ ((packed)) msg_get_param_t;

typedef struct {
  uint32_t param_value;
  uint8_t param_id;
} __attribute__ ((packed)) msg_set_param_t;


///////////////////////// Response Packet Structures /////////////////////////////////
// All response packets begin with a single byte response character, not included in the
// structs below because it would mess with efficient packing on 32-bit microcontrollers.

typedef struct {
  uint16_t slave_hw_ver;
  uint16_t slave_fw_ver;
  uint16_t queue_depth;
} __attribute__ ((packed)) rsp_initialize_t;

typedef struct {
  uint16_t queued_moves;
  imc_axis_error status;
} __attribute__ ((packed)) rsp_status_t;


//typedef struct __attribute__ ((__packed__)){
//} __attribute__ ((packed)) rsp_queue_move_t;

typedef struct {
  uint32_t value;
} __attribute__ ((packed)) rsp_get_param_t;

//typedef struct {
//} __attribute__ ((packed)) rsp_set_param_t;


#define IMC_MESSAGE_TYPE_COUNT 7
extern const uint8_t imc_message_length[IMC_MESSAGE_TYPE_COUNT + 1]; 
extern const uint8_t imc_resp_length[IMC_MESSAGE_TYPE_COUNT + 1];

#endif


