#ifndef __MODE_FULL_LOOP_H
#define __MODE_FULL_LOOP_H

#include <stdint.h>
#include "drivers/line_tracker.h"

typedef enum {
    ABCD_IDLE = 0,
    ABCD_PRETILT,
    ABCD_STRAIGHT_1,
    ABCD_ARC_1,
    ABCD_STRAIGHT_2,
    ABCD_ARC_2,
    ABCD_FINAL_BALANCE,
    ABCD_DONE
} ABCD_State;

void ABCD_Init(void);
void ABCD_OnStartStop(void);
void ABCD_Stop(void);
void ABCD_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

uint8_t ABCD_IsRunning(void);
ABCD_State ABCD_GetState(void);
float ABCD_GetDistanceMm(void);
int8_t ABCD_GetDrivePercent(void);
int8_t ABCD_GetCurveDirection(void);

#endif
