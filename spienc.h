/* SPI Encoder Reading module */
#ifndef __spienc_h
#define __spienc_h

#ifndef USE_QD_ENC

void enc_Init(void);
void enc_idle(void);

uint8_t get_enc_value(volatile int32_t *);
void set_enc_value(int32_t);

#endif
#endif