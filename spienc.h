/* SPI Encoder Reading module */
#ifndef __qdenc_h
#define __qdenc_h

void spi_init(void);


int32_t get_spienc_value(void);
void set_spienc_value(int32_t);


#endif