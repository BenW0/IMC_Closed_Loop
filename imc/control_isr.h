#ifndef control_isr_h
#define control_isr_h

#define SYNC_TIMEOUT 4800000 // sync timeout defaults to 100 ms
#define SYNC_DELAY 96000 // Two MS

void trigger_sync_delay(void);
void enable_sync_interrupt(void); 
// Tri-state sync pin, but don't enable any interrupt
void float_sync_line(void);

#endif
