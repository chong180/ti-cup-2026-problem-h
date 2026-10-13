#ifndef __MODE_LOOP_ARC_H
#define __MODE_LOOP_ARC_H

#include <stdint.h>
#include "drivers/line_tracker.h"

/*
 * Mode 006: mode6 trajectory (arc return to A) + 50cm straight line tracking.
 *
 * State flow:
 *   IDLE -> HOLD -> DRIVE (mode6 arc trajectory, 0..5600mm)
 *        -> TRACK (extra 500mm line tracking, 5600..6100mm)
 *        -> FINAL (ball settle) -> DONE
 */

typedef enum {
    MODE006_IDLE = 0,
    MODE006_HOLD,
    MODE006_DRIVE,
    MODE006_TRACK,
    MODE006_FINAL,
    MODE006_DONE
} Mode006_State;

void Mode006_Init(void);
void Mode006_OnStartStop(void);
void Mode006_Stop(void);
void Mode006_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

uint8_t Mode006_IsRunning(void);
Mode006_State Mode006_GetState(void);
float Mode006_GetTargetMm(void);
float Mode006_GetDistanceMm(void);
float Mode006_GetFinishMm(void);
int8_t Mode006_GetDrivePercent(void);

#endif
