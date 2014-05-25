#include "protocol/message_structs.h"
#include "protocol/constants.h"
#include "parser.h"
#include "utils.h"
#include "hardware.h"

#include <usb_serial.h>
#include <pin_config.h>
#include <mk20dx128.h>

uint8_t txBuffer[BUFFER_LENGTH];
volatile uint32_t txBufferLength;
volatile uint32_t txBufferIndex;
volatile uint32_t irqcount;

volatile parser_state_t parser;
generic_response response;

void initialize_i2c(uint8_t addr){
  SIM_SCGC4 |= SIM_SCGC4_I2C0;
  SDA_CTRL = PORT_PCR_MUX(2) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;
  SCL_CTRL = PORT_PCR_MUX(2) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;
#if F_BUS == 48000000
  I2C0_F = 0x27;
  I2C0_FLT = 4;
#elif F_BUS == 24000000
  I2C0_F = 0x1F;  
  I2C0_FLT = 2;
#else
#error "F_BUS must be 48 MHz or 24 MHz"
#endif
  I2C0_C2 = I2C_C2_HDRS;
  I2C0_C1 = I2C_C1_IICEN;
  I2C0_A1 = addr << 1;
  I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE;
}

void initialize_parser(void){
  parser.status = PARSER_EMPTY;
  parser.packet_type = 0; // 0 is guaranteed to not be a packet type byte
  parser.head = (uint8_t*) &parser.packet;
  vmemset(&parser.packet, 0, sizeof(parser.packet));
}

void feed_data(uint8_t input){
  if(parser.status == PARSER_NEW_EVENT || parser.status == PARSER_ERR){
    // Haven't dealt with the event soon enough, and we have no queue,
    // so this is an error. I2C shouldn't be prone to this, however
    parser.status = PARSER_ERR; 
    return;
  }
  if(0 == parser.packet_type){
    imc_message_type type = (imc_message_type) input;
    if(type < IMC_MSG_INITIALIZE || IMC_MSG_QUICKSTOP < type){ // First and last members of imc_message_type
      parser.status = PARSER_ERR;
      return;
    }
    parser.packet_type = type;
    parser.remaining = imc_message_length[type] + 1; // Include an extra for the checksum
    parser.head = (uint8_t*) &parser.packet;

    if(parser.remaining > PROTOCOL_MAX_TRANSMIT_SIZE){
      parser.big_packet = 1;
    }
    
    return;
  }
  if(parser.remaining-- > 0){
    //   usb_serial_putchar(input + 'a');
    *parser.head++ = input;
  }
  // If we've finished an entire packet, signal that
  if(parser.remaining == 0){
    //   usb_serial_putchar('D');
    uint8_t sum = parser.packet_type; 
    uint32_t size = imc_message_length[sum];
    uint32_t i;
    for(i = 0; i < size; i++){
      //   usb_serial_putchar(((uint8_t*) &parser.packet)[i] + 'a');
      sum ^= ((uint8_t*) &parser.packet)[i];
    }

    // usb_serial_putchar('0' + size);
      
    if(sum == (((uint8_t*) &parser.packet)[size])){
      //   usb_serial_putchar('K');
      parser.status = PARSER_NEW_EVENT;
    }else{
      //     usb_serial_putchar('E');
    
      parser.status = PARSER_ERR;
    }
  }
}

void send_response(imc_response_type status,uint32_t size){
  uint8_t checksum = status;
  uint32_t i;
  txBufferLength = size+2;
  txBuffer[0] = status;

  for(i = 0; i < size; i++){
    uint8_t byte = ((uint8_t*) &response)[i];
    checksum ^= byte;
    txBuffer[i+1] = byte;
  }
  txBuffer[size+1] = checksum;
}

void i2c0_isr(void)
{
  uint8_t status, c1, data;
  static uint8_t receiving=0;

  I2C0_S = I2C_S_IICIF;
  status = I2C0_S;
  //serial_print(".");
  if (status & I2C_S_ARBL) {
    // Arbitration Lost
    I2C0_S = I2C_S_ARBL;
    if (!(status & I2C_S_IAAS)) return;
  }
  if (status & I2C_S_IAAS) {
    // Addressed As A Slave
    if (status & I2C_S_SRW) {
      I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_TX;
      I2C0_D = txBuffer[0];
      txBufferIndex = 1;
    } else {
      // Begin Slave Receive
      receiving = 1;
      I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE;
      // So for some reason that I absolutely don't understand, ignoring this byte actually is fine.
      // What is this byte? I should run a test.
      data = I2C0_D;
      if(parser.big_packet){
	parser.big_packet = 0; // We're in the second transmision of a big packet, so we don't reinitialize the parser.
      }else{
	// Reset the parser state, but don't really bother to memset the data buffer
	parser.status = PARSER_EMPTY;
	parser.packet_type = 0; // 0 is guaranteed to not be a packet type
	parser.head = (uint8_t*) &parser.packet;
      }
    }
    I2C0_S = I2C_S_IICIF;
    return;
  }
  c1 = I2C0_C1;
  if (c1 & I2C_C1_TX) {
    // Continue Slave Transmit
    if ((status & I2C_S_RXAK) == 0) {
      // Master ACK'd previous byte
      if (txBufferIndex < txBufferLength) {
	I2C0_D = txBuffer[txBufferIndex++];
      } else {
	I2C0_D = 0; // Pad with zeros
      }
      I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_TX;
    } else {
      I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE;
      data = I2C0_D;
    }
  } else {
    // Continue Slave Receive
    irqcount = 0;    
    SDA_CTRL = (~IRQC_MASK & SDA_CTRL) | IRQC_RISING;
    data = I2C0_D;
    if(receiving)
      feed_data(data);
  }
}
 
