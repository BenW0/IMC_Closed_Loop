/********************************************************************************
 * RawHID Messaging Module
 * Ben Weiss, University of Washington 2014
 * Purpose: Handles the RawHID interface for sending and receiving commands from
 * my desktop-based rawhid_listener program. Basically, data is sent out in one
 * of two forms: Debug and text stream data is written using hid_printf
 * or hid_print, which is split into packet-sized pieces, given a header, and 
 * sent out over the rawhid interface. Binary packet data can be sent using
 * hid_write or hid_write_packet. hid_write breaks the data into packets and
 * appends a header; hid_write_packet just writes the raw packet data to the rawhid
 * interface.
 *
 * Received packets can be read through hid_read(), which filters for a special
 * header requesting a device ID, used to distinguish between multiple Teensy's connected
 * to the same computer and running the same code.
 * 
 * Source: 
 * 
 * License: None. This is for internal development only!
 ********************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// Note: This module does not include common.h. This is supposed to be a standalone resource...

#include <usb_rawhid.h>
#include <usb_seremu.h>
#include <usb_desc.h>
#include "rawhid_msg.h"
#include "imc/utils.h"    // for vmemset

// Constants ===========================================================


// Local Variables =====================================================
char message[200];
uint8_t pack[RAWHID_TX_SIZE];
uint32_t pack_len = 0;
uint32_t devid = 0;      // device ID



// this function sets up the rawhid module. Providing a device_id allows the host computer
// to differentiate between teensys running the same code connected at the same time.
void hid_init(uint32_t device_id)
{
  devid = device_id;
  vmemset(pack, 0, RAWHID_TX_SIZE);
  pack_len = 0;
}


// This code is not terribly robust; it may block while waiting for a packet to
// buffer for up to 100 ms, and it doesn't know how to recover from lost packets
void hid_printf(const char *str, ...)
{
  va_list args;
  va_start(args, str);
  vsprintf(message, str, args);
  hid_print(message, strlen(message), 100);
  va_end(args);
}

// header specifies a single-byte header to be appended at the start of each
// packet, and should not be 0 or 1, which are reserved. If this function
// is called successively with the same header, data is collapsed into as few
// packets as possible. To ensure packets have all been sent, use hid_flush();
// Returns 1 if successful, 0 if unable to send before timeout. A 0 return may
// still have sent or cached some of the packet data to be sent in a future packet.
int hid_write(uint8_t header, const uint8_t *data, uint32_t count, uint32_t timeout)
{
  // check -- is there data waiting to be written?
  if(pack_len)
  {
    // does the data waiting have the same header?
    if(header != pack[0])
    {
      // fail if we couldn't flush the buffered packet!
      if(!hid_flush(timeout))
        return 0;
      pack[0] = header;
      pack_len = 1;
    }
  }
  else
  {  
    pack[0] = header;
    pack_len = 1;
  }

  // append the data to the packet(s)
  for(uint32_t i = 0; i < count; i++)
  {
    pack[pack_len++] = data[i];
    if(RAWHID_TX_SIZE == pack_len)    // packet full!
    {
      if(!usb_rawhid_send((void *)pack, timeout))
        return 0;
      pack_len = 1;
      pack[0] = header;
    }
  }

  // if we ended on an even packet, reset the pack buffer. Otherwise we'll send an empty
  // packet if the next call has a different header.
  if(1 == pack_len)
    pack_len = 0;

  return 1;
}


// flushes the local not-full packet if one is waiting to be sent.
int hid_flush(uint32_t timeout)
{
  if(!pack_len)
    return;   // nothing to send
  // send a partially-full packet.
  vmemset(pack + pack_len, 0, RAWHID_TX_SIZE - pack_len);
  if(usb_rawhid_send(pack, timeout))
  {
    pack_len = 0;
    return 1;
  }
  else
    return 0;   // keep the packet on the buffer.
}

// buf should be a buffer to write to, 64 bytes long. Timeout is the time to wait
// (in ms) before failing the read. If the read times out (no data available), 0 is returned,
// if it succeeds, the number of bytes read (always 64) is returned. Even if the
// read fails, buf may be overwritten!
int hid_read(uint8_t *buf, uint32_t timeout)
{
  if(!usb_rawhid_recv((void*)buf, timeout))
    return 0;
  
  // check for device id request packet. The header is repeated 3x just to make sure this
  // isn't a normal packet...
  while(RX_HEAD_DEVID == buf[0] && RX_HEAD_DEVID == buf[1] && RX_HEAD_DEVID == buf[2])
  {
    // write back our device ID:
    buf[0] = TX_HEAD_DEVID;
    for(uint8_t i = 0; i < 3; i++)
      buf[i+1] = (uint8_t)((devid >> (i*8)) & 0xFF);
    usb_rawhid_send(buf, timeout);     // if the buffer's full, oh well.
    
    // read another packet
    if(!usb_rawhid_recv((void*)buf, timeout))
      return 0;
  }

  // this packet can be returned
  return 1;
}
