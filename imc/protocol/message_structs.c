#include "message_structs.h"
  
const uint8_t imc_message_length[IMC_MESSAGE_TYPE_COUNT + 1] = {0, sizeof(msg_initialize_t), 
                              0/*sizeof(msg_status_t)*/, 0/*sizeof(msg_home_t)*/, sizeof(msg_queue_move_t), 
                              sizeof(msg_get_param_t), sizeof(msg_set_param_t), 0};

const uint8_t imc_resp_length[IMC_MESSAGE_TYPE_COUNT + 1] = {0, sizeof(rsp_initialize_t), 
			      sizeof(rsp_status_t), 0, 0/*sizeof(rsp_queue_move_t)*/,
                              sizeof(rsp_get_param_t), 0/*sizeof(rsp_set_param_t)*/, 0};
