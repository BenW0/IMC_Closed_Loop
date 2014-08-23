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
#include "hardware.h"
#include "stepper.h"
#include "queue.h"
#include "parameters.h"
#include "control_isr.h"
#include "parser.h"
#include <pin_config.h>
//||\\!! TEMP
//#include "../common.h"

void portb_isr(void){
  if(SDA_CTRL & ISF){
    SDA_CTRL |= ISF;
    if (!(I2C0_S & I2C_S_BUSY)){
      SDA_CTRL = (~IRQC_MASK & SDA_CTRL) | IRQC_NONE;
      // We're done right here
    } else {
      if (++irqcount >= 2) {
	SDA_CTRL = (~IRQC_MASK & SDA_CTRL) | IRQC_NONE;
      }
    }
  }
  
  //||\\!! TODO: This safety feature disabled for now because I have a gentler handling in stepper_hook.
  if(MIN_LIMIT_CTRL & ISF){
    if(parameters.homing &  ENABLE_MIN){
      //stop_motion();
      //st.state = STATE_ERROR;
      //parameters.error_low = IMC_ERR_MECHANICAL;
    }
    MIN_LIMIT_CTRL |= ISF;
  }
  if(MAX_LIMIT_CTRL & ISF){
    if(parameters.homing &  ENABLE_MAX){
      //stop_motion();
      //st.state = STATE_ERROR;
      //parameters.error_low = IMC_ERR_MECHANICAL;
    }
    MAX_LIMIT_CTRL |= ISF;
  }
  if(SYNC_CTRL & ISF){
    //||\\!! temp
    //hid_printf("%u-%i sync pin isr. state = %i\n", get_systick_tenus(), (CONTROL_DDR & SYNC_BIT) ? -1 : CONTROL_PORT(DIR) & SYNC_BIT, st.state);
    switch(st.state){
    case STATE_SYNC:
      {
	uint32_t timer_value;
	timer_value = SYNC_TIMEOUT - PIT_CVAL2; // Figure out how far we've gotten
	PIT_TCTRL2 &= ~TEN; // Stop counting down
	parameters.sync_error = timer_value;
  
    //||\\!! temp
    //hid_printf("%u-%i  sync_error = %i\n", get_systick_tenus(), (CONTROL_DDR & SYNC_BIT) ? -1 : CONTROL_PORT(DIR) & SYNC_BIT, timer_value);
      }
      execute_move();
      break;
    case STATE_IDLE:
      if(queue_length() > 0){
	execute_move();
	break;
      }
      break;  //||\\!! FIXME - this break added to keep from erroring while debugging the control flow.
      // Otherwise fall through and set an error condition
    case STATE_EXECUTE:
      // This should never happen - if we're in execute, we should hold the line low and not enable this interrupt...
      st.state = STATE_ERROR;
      parameters.error_low = IMC_ERR_TIMEOUT; // Maybe not best error code, but...
    default:
      ; // Just handle this...
    }
    SYNC_CTRL = (SYNC_CTRL & ~IRQC_MASK) | IRQC_NONE;
    SYNC_CTRL |= ISF;
  }
}

void pit2_isr(void){
  PIT_TFLG2 = 1;
  // Stop the timer...
  PIT_TCTRL2 &= ~TEN;

  if(PIT_LDVAL2 != SYNC_DELAY){
    //||\\!! temp
    //hid_printf("%u-%i pit2 timeout\n", get_systick_tenus(), (CONTROL_DDR & SYNC_BIT) ? -1 : CONTROL_PORT(DIR) & SYNC_BIT);
    // We've timed out and bad things are happening   
    return;//||\\!! FIXME - For debugging I don't want a timeout error, so I am disabling this code!
    st.state = STATE_ERROR;
    parameters.error_low = IMC_ERR_TIMEOUT; // Maybe not best error code, but...
    stop_motion();
  }else{
    CONTROL_DDR |= SYNC_BIT;
    //||\\!! temp
    //hid_printf("%u-%i Sync low\n",get_systick_tenus(), (CONTROL_DDR & SYNC_BIT) ? -1 : CONTROL_PORT(DIR) & SYNC_BIT);
  }
}

void trigger_sync_delay(void){
  PIT_TCTRL2 &= TEN; // Stop the timer 
  PIT_LDVAL2 = SYNC_DELAY; // Switch to triggering at the end of our propogation window
  PIT_TCTRL2 |= TEN | TIE; // Start counting down 
  //||\\!! temp
  //hid_printf("%u-%i Trigger sync delay\n", get_systick_tenus(), (CONTROL_DDR & SYNC_BIT) ? -1 : CONTROL_PORT(DIR) & SYNC_BIT);
}
void enable_sync_interrupt(void){
  CONTROL_DDR &= ~SYNC_BIT;
  SYNC_CTRL = MUX_GPIO | IRQC_ONE;
    //||\\!! temp
    //hid_printf("%u-%i Sync float\n", get_systick_tenus(), (CONTROL_DDR & SYNC_BIT) ? -1 : CONTROL_PORT(DIR) & SYNC_BIT);
}

void float_sync_line(void){
  CONTROL_DDR &= ~SYNC_BIT;
  SYNC_CTRL = MUX_GPIO | IRQC_NONE;
}
