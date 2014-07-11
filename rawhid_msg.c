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
 * All outgoing packets contain a header, the first two bits of which are a packet type
 * identifier (0b00 is text, 0b01-0b11 are user-specified data channels), followed by 6 bits 
 * indicating the packet length (0-63). A special packet header, 0b11111100 (=text packet, 
 * length 64) is a special packet used to retrieve the device ID. 
 *
 * Received packets can be read through hid_read(), which filters for a special
 * header requesting a device ID, used to distinguish between multiple Teensy's connected
 * to the same computer and running the same code. Except for that special header (0x08 = 
 * backspace), incoming packets are not expected to have a header.
 *
 * IF the device is configured to operate as a serial device (-DUSB_SERIAL instead of
 * -DUSB_RAWHID the Makefile), this module sends all text to the host over serial without a header.
 * Data streams each get a header including length information as before. This feature
 * has not been tested.
 * 
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

#ifdef USB_RAWHID
#include <usb_rawhid.h>
#include <usb_seremu.h>
#else
#include <usb_serial.h>
#endif

#include <usb_desc.h>
#include "rawhid_msg.h"
#include "imc/utils.h"    // for vmemset

// Constants ===========================================================


// Local Variables =====================================================
char message[200];
uint8_t pack[HID_PACKLEN];
uint32_t pack_len = 0;
uint32_t devid = 0;      // device ID



// this function sets up the rawhid module. Providing a device_id allows the host computer
// to differentiate between teensys running the same code connected at the same time.
void hid_init(uint32_t device_id)
{
  devid = device_id;
  vmemset(pack, 0, HID_PACKLEN);
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
int hid_write(hid_pack_type pack_type, const uint8_t *data, uint32_t count, uint32_t timeout)
{
#ifndef USB_RAWHID
  // shortcut for text streams -- just fire them off
  if(TX_PACK_TYPE_TEXT == pack_type)
  {
    usb_serial_write(data, count);
    return 1;
  }
#endif

  // check -- is there data waiting to be written?
  if(pack_len)
  {
    // does the data waiting have the same header?
    if(pack_type != (pack[0] & TX_PACK_TYPE_MASK))
    {
      // fail if we couldn't flush the buffered packet!
      if(!hid_flush(timeout))
        return 0;
      pack[0] = pack_type;
      pack_len = 1;
    }
  }
  else
  {  
    pack[0] = pack_type;
    pack_len = 1;
  }

  // append the data to the packet(s)
  for(uint32_t i = 0; i < count; i++)
  {
    pack[pack_len++] = data[i];
    if(HID_PACKLEN == pack_len)    // packet full!
    {
      // complete by inserting the length into the packet header
      pack[0] |= HID_PACKLEN << 2;
      if(!hid_write_packet((void *)pack, timeout))
        return 0;
      pack_len = 1;
      pack[0] = pack_type;
    }
  }

  // if we ended on an even packet, reset the pack buffer. Otherwise we'll send an empty
  // packet if the next call has a different header.
  if(1 == pack_len)
    pack_len = 0;

  return 1;
}

// data contains the packet to send; timeout is the number of ms to wait for
// space on the tx queue before failing (--> return 0). Don't use this to directly
// send packets as a suer. Use hid_print or hid_write instead.
__attribute__ ((always_inline)) inline int hid_write_packet(const void *data, uint32_t timeout)
{
#ifdef USB_RAWHID
  return usb_rawhid_send(data, timeout);
#else
  return usb_serial_write(data, HID_PACKLEN);
#endif
}

// flushes the local not-full packet if one is waiting to be sent.
int hid_flush(uint32_t timeout)
{
  if(!pack_len)
    return 1;   // nothing to send
  // send a partially-full packet.
  vmemset(pack + pack_len, 0, HID_PACKLEN - pack_len);
  // complete the header by adding the packet length
  pack[0] |= pack_len << 2;
#ifdef USB_RAWHID
  if(usb_rawhid_send(pack, timeout))
#else
  if(usb_serial_write(pack, pack_len))
#endif
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
#ifdef USB_RAWHID
  if(!usb_rawhid_recv((void*)buf, timeout))
    return 0;
#else
  if(!usb_serial_read((void*)buf, HID_PACKLEN))
    return 0;
#endif
  
  // check for device id request packet. The header is repeated 3x just to make sure this
  // isn't a normal packet...
  while(RX_HEAD_DEVID == buf[0] && RX_HEAD_DEVID == buf[1] && RX_HEAD_DEVID == buf[2])
  {
    // write back our device ID:
    buf[0] = TX_HEAD_DEVID;
    for(uint8_t i = 0; i < 3; i++)
      buf[i+1] = (uint8_t)((devid >> (i*8)) & 0xFF);
    hid_write_packet(buf, timeout);     // if the buffer's full, oh well.
    
    // read another packet
#ifdef USB_RAWHID
    if(!usb_rawhid_recv((void*)buf, timeout))
      return 0;
#else
    if(!usb_serial_read((void*)buf, HID_PACKLEN))
      return 0;
#endif
  }

  // this packet can be returned
  return 1;
}

int hid_available(void)
{
#ifdef USB_RAWHID
  return usb_rawhid_available();
#else
  return usb_serial_available();
#endif
}