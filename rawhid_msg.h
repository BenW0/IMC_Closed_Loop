/* rawhid message processor header
*/

#ifndef rawhid_msg_h
#define rawhid_msg_h

#include <inttypes.h>
#include <stdarg.h>


// Message header constants, shared with rawhid_listener
#define RX_HEAD_DEVID     0x08      // this is backspace, and shouldn't appear in a normal text transmission...
#define TX_HEAD_DEVID     0x00
#define TX_HEAD_TEXT      0x01


void hid_init(uint32_t device_id);

// This code is not terribly robust; it may block while waiting for a packet to
// buffer for up to 100 ms, and it doesn't know how to recover from lost packets
void hid_printf(const char *str, ...);

// header specifies a single-byte header to be appended at the start of each
// packet, and should not be 0 or 1, which are reserved. If this function
// is called successively with the same header, data is collapsed into as few
// packets as possible. To ensure packets have all been sent, use hid_flush();
// Returns 1 if successful, 0 if unable to send before timeout. A 0 return may
// still have sent or cached some of the packet data to be sent in a future packet.
int hid_write(uint8_t header, const uint8_t *data, uint32_t count, uint32_t timeout);

// flushes the local not-full packet if one is waiting to be sent.
int hid_flush(uint32_t timeout);




// prints text to the hid interface. timeout is the number of ms to wait for the usb module
// to queue the packet before failing. Returns 0 if timeout occurred, 1 otherwise. 0 may still
// imply some of the text was sent or will be sent in a future packet.
__attribute__ ((always_inline)) inline int hid_print(const char *data, uint32_t count, uint32_t timeout)
{
  return hid_write(TX_HEAD_TEXT, (const uint8_t *)data, count, timeout);
}

// data contains the packet to send; timeout is the number of ms to wait for
// space on the tx queue before failing (--> return 0). Don't start your packet
// with 0x0 or 0x1 as the first byte or things will get confused!.
__attribute__ ((always_inline)) inline int hid_write_packet(const void *data, uint32_t timeout)
{
  return usb_rawhid_send(data, timeout);
}

// buf should be a buffer to write to, 64 bytes long. Timeout is the time to wait
// (in ms) before failing the read. If the read times out (no data available), 0 is returned,
// if it succeeds, the number of bytes read (always 64) is returned. Even if the
// read fails, buf may be overwritten!
int hid_read(uint8_t *buf, uint32_t timeout);

__attribute__ ((always_inline)) inline int hid_avail(void)
{
  return usb_rawhid_available();
}


#endif