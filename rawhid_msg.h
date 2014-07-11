/* rawhid message processor header
*/

#ifndef rawhid_msg_h
#define rawhid_msg_h

#include <inttypes.h>
#include <stdarg.h>

#ifdef USB_RAWHID
#define HID_PACKLEN     RAWHID_TX_SIZE
#else
#define HID_PACKLEN     64
#endif


// Message header constants, shared with rawhid_listener
#define RX_HEAD_DEVID      0x08      // this is backspace, and shouldn't appear in a normal text transmission...
#define TX_HEAD_DEVID           0xFC      // for querying the deviceid, this is the entire header (which is the same as an impossible 64-length text packet)

typedef enum {
  TX_PACK_TYPE_TEXT = 0x00,
  TX_PACK_TYPE_DATA0 = 0x01,
  TX_PACK_TYPE_DATA1 = 0x02,
  TX_PACK_TYPE_DATA2 = 0x03
} __attribute__ ((packed)) hid_pack_type;

#define TX_PACK_TYPE_MASK     0x03


void hid_init(uint32_t device_id);

// This code is not terribly robust; it may block while waiting for a packet to
// buffer for up to 100 ms, and it doesn't know how to recover from lost packets
void hid_printf(const char *str, ...);

// pack_type specifies the packet type and is . If this function
// is called successively with the same header, data is collapsed into as few
// packets as possible. To ensure packets have all been sent, use hid_flush();
// Returns 1 if successful, 0 if unable to send before timeout. A 0 return may
// still have sent or cached some of the packet data to be sent in a future packet.
int hid_write(hid_pack_type pack_type, const uint8_t *data, uint32_t count, uint32_t timeout);

// flushes the local not-full packet if one is waiting to be sent.
int hid_flush(uint32_t timeout);




// prints text to the hid interface. timeout is the number of ms to wait for the usb module
// to queue the packet before failing. Returns 0 if timeout occurred, 1 otherwise. 0 may still
// imply some of the text was sent or will be sent in a future packet.
__attribute__ ((always_inline)) inline int hid_print(const char *data, uint32_t count, uint32_t timeout)
{
  return hid_write(TX_PACK_TYPE_TEXT, (const uint8_t *)data, count, timeout);
}

// buf should be a buffer to write to, 64 bytes long. Timeout is the time to wait
// (in ms) before failing the read. If the read times out (no data available), 0 is returned,
// if it succeeds, the number of bytes read (always 64) is returned. Even if the
// read fails, buf may be overwritten!
int hid_read(uint8_t *buf, uint32_t timeout);


int hid_available(void);


#endif