#include "hardware.h"
#include "stepper.h"
#include "queue.h"
#include "parameters.h"
#include "control_isr.h"
#include "parser.h"
#include <pin_config.h>

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
  

  if(MIN_LIMIT_CTRL & ISF){
    if(parameters.homing &  ENABLE_MIN){
      stop_motion();
      st.state = STATE_ERROR;
      parameters.error_low = IMC_ERR_MECHANICAL;
    }
    MIN_LIMIT_CTRL |= ISF;
  }
  if(MAX_LIMIT_CTRL & ISF){
    if(parameters.homing &  ENABLE_MAX){
      stop_motion();
      st.state = STATE_ERROR;
      parameters.error_low = IMC_ERR_MECHANICAL;
    }
    MAX_LIMIT_CTRL |= ISF;
  }
  if(SYNC_CTRL & ISF){
    switch(st.state){
    case STATE_SYNC:
      {
	uint32_t timer_value;
	timer_value = SYNC_TIMEOUT - PIT_CVAL2; // Figure out how far we've gotten
	PIT_TCTRL2 &= ~TEN; // Stop counting down
	parameters.sync_error = timer_value;
      }
      execute_move();
      break;
    case STATE_IDLE:
      if(queue_length() > 0){
	execute_move();
	break;
      }
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
    // We've timed out and bad things are happening   
    st.state = STATE_ERROR;
    parameters.error_low = IMC_ERR_TIMEOUT; // Maybe not best error code, but...
    stop_motion();
  }else{
    CONTROL_DDR |= SYNC_BIT;
  }
}

void trigger_sync_delay(void){
  PIT_TCTRL2 &= TEN; // Stop the timer 
  PIT_LDVAL2 = SYNC_DELAY; // Switch to triggering at the end of our propogation window
  PIT_TCTRL2 |= TEN | TIE; // Start counting down 
}
void enable_sync_interrupt(void){
  CONTROL_DDR &= ~SYNC_BIT;
  SYNC_CTRL = MUX_GPIO | IRQC_ONE;
}

void float_sync_line(void){
  CONTROL_DDR &= ~SYNC_BIT;
  SYNC_CTRL = MUX_GPIO | IRQC_NONE;
}
