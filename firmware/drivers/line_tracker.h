#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdint.h>

typedef struct {
    uint8_t activeMask; /* Bit 0..7: corresponding sensor sees the black line. */
    int8_t error;       /* Negative: line left; positive: line right. */
    uint8_t lineFound;
} LineTracker_State;

void LineTracker_Read(LineTracker_State *state);

#endif
