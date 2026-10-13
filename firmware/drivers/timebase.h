#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/* Shared 1 ms system timebase used by keys, encoders and run timing. */
void Timebase_Init(void);
uint32_t Timebase_GetMillis(void);
uint8_t Timebase_TakeBallControlTick(void);

#endif
