/* SPI Encoder Reading module */
#ifndef __qdenc_h
#define __qdenc_h

#ifndef USE_QD_ENC

void enc_Init(void);

int32_t get_enc_value(void);
void set_enc_value(int32_t);

#endif
#endif