/* Quadrature Encoder Reading module */
#ifndef __qdenc_h
#define __qdenc_h

void QEI_Init(void);
void QEI_Deinit(void);


int32_t get_qenc_value(void);
void set_qenc_value(int32_t);

extern volatile unsigned long isr1_count, isr2_count;

#endif