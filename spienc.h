/* SPI Encoder Reading module */
#ifndef __spienc_h
#define __spienc_h

#ifndef USE_QD_ENC

// comment the following define to bitbang the interface to the encoder instead of using the SPIFIFO module
// (useful for when SPI is being used for another task)
#define ENC_USE_SPIFIFO

#ifndef ENC_USE_SPIFIFO
// pins to use for serial communications
// Pin 23 is sck
// Pin 21 is din
// Pin 22 is cs
#define ENC_SCK_PORT(reg) GPIOC_P##reg
#define ENC_SCK_CTRL PORTC_PCR2
#define ENC_SCK_BIT (1<<2)

#define ENC_DIN_PIN  6
#define ENC_DIN_PORT(reg) GPIOD_P##reg
#define ENC_DIN_CTRL PORTD_PCR6
#define ENC_DIN_BIT (1<<6)

#define ENC_CS_PORT(reg) GPIOC_P##reg
#define ENC_CS_CTRL PORTC_PCR1
#define ENC_CS_BIT (1<<1)

// Utility functions to make bitbanging a bit cleaner
#define SCK_ON()   ENC_SCK_PORT(SOR) = ENC_SCK_BIT;
#define SCK_OFF()  ENC_SCK_PORT(COR) = ENC_SCK_BIT;
#define CS_ON()    ENC_CS_PORT(SOR) = ENC_CS_BIT;
#define CS_OFF()   ENC_CS_PORT(COR) = ENC_CS_BIT;
#define DIN_READ() ((ENC_DIN_PORT(DIR) & ENC_DIN_BIT) >> ENC_DIN_PIN)

#else // ENC_USE_SPFIFO

// a few dummys so things compile even when not using this functionality
#define SCK_ON()
#define SCK_OFF()
#define CS_ON()
#define CS_OFF()
#define DIN_READ()  0

#endif  // !ENC_SUE_SPFIFO

void enc_Init(void);
void enc_idle(void);

uint8_t get_enc_value(volatile int32_t *);
void set_enc_value(int32_t);

bool enc_lost_track(void);

#endif
#endif