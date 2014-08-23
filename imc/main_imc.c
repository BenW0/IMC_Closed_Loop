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
#include "parser.h"
#include "queue.h"
#include "protocol/constants.h"
#include "protocol/message_structs.h"
#include "hardware.h"
#include "parameters.h"
#include "stepper.h"
#include "control_isr.h"
#include "config.h"
#include "peripheral.h"
#include "homing.h"
#include <usb_serial.h>
#include <mk20dx128.h>
#include <pin_config.h>

// used to be main()
void imc_init(void){
  configure_nvic();
  initialize_i2c(I2C_BASE_ADDRESS + read_i2c_address());
  // Configure all of the hardware and internal state
  initialize_motion_queue();
  initialize_parser();
  reset_parameters();
  initialize_stepper_state();
  set_microstepping(DEFAULT_MICROSTEPPING);
  reset_hardware();
  float_sync_line();
}

// used to be while(1) in main()
void imc_idle(void)
{
  if(parser.status == PARSER_ERR){
    send_response(IMC_RSP_UNKNOWN,0);
    initialize_parser();
    return;
  }
  if(parser.status == PARSER_NEW_EVENT){
    switch(parser.packet_type){
    case IMC_MSG_INITIALIZE:
      initialize_motion_queue();
      initialize_parser();
      // Unlike out first initialization round, don't reset parameters
      initialize_stepper_state();
      float_sync_line();
      response.init.slave_hw_ver = 0;
      response.init.slave_fw_ver = 0;
      response.init.queue_depth = MOTION_QUEUE_LENGTH;
      send_response(IMC_RSP_OK,sizeof(rsp_initialize_t));
      break;
    case IMC_MSG_GETPARAM:
      handle_get_parameter(&parser.packet.get_param, &response.param);
      send_response(IMC_RSP_OK,sizeof(rsp_get_param_t));
      break;
    case IMC_MSG_SETPARAM:
      handle_set_parameter(&parser.packet.set_param);
      send_response(IMC_RSP_OK,0);
      break;	
    case IMC_MSG_QUEUEMOVE:
      {
        int space = enqueue_block(&parser.packet.move);
        send_response(space < 0 ? IMC_RSP_QUEUEFULL : IMC_RSP_OK,0);
        // If we're adding moves in idle state, make sure that the sync interface is listening
        if(st.state == STATE_IDLE)
          enable_sync_interrupt();
      }
      break;
    case IMC_MSG_STATUS:
      response.status.queued_moves = queue_length();
      if(st.state == STATE_ERROR){
        response.status.status = parameters.error_low;
      }else{
        response.status.status = IMC_ERR_NONE;
      }
      send_response(IMC_RSP_OK,sizeof(rsp_status_t));
      break;
    case IMC_MSG_HOME:
      enter_homing_routine();
      break;
    case IMC_MSG_QUICKSTOP:
      send_response(IMC_RSP_ERROR,0);
      break;
    }
    initialize_parser();
  }
}
