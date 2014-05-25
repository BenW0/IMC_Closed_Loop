/* Quadrature Encoder Reading module */
#ifndef __qdenc_h
#define __qdenc_h

#ifdef USE_QD_ENC

void enc_Init(void);
void enc_Deinit(void);

//void enc_idle(void) {}    // not used by quadrature encoder

uint8_t get_enc_value(volatile int32_t *);
void set_enc_value(int32_t);

extern volatile unsigned long isr1_count, isr2_count;

#endif
#endif