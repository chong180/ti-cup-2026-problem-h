#include "ball_balance/ball_route.h"

#include <stdint.h>

#include "ball_balance/ball_zero_control.h"
#include "ball_balance/ball_balance.h"

#define ROUTE_ZERO_TARGET_MM             (0.0f)
#define ROUTE_POSITIVE_TARGET_MM         (50.0f)
#define ROUTE_NEGATIVE_TARGET_MM         (-50.0f)

/* All confirmations are counted on fresh 60 FPS camera frames. */
#define ROUTE_ZERO_POSITION_TOL_MM       (2.0f)
#define ROUTE_ZERO_SPEED_TOL_MM_S        (8.0f)
#define ROUTE_ZERO_STABLE_FRAMES         (10U)

/* Measured braking travel is about 11 mm.  Switch at +3.9 cm so inertia
 * carries the ball just past the required +5 cm waypoint.  Do not gate
 * this transition with differentiated velocity: coordinate noise can
 * keep the estimated speed nonzero and strand the state at TO_POSITIVE. */
#define ROUTE_POSITIVE_SWITCH_MM         (39.0f)

#define ROUTE_FINAL_POSITION_TOL_MM      (2.0f)
#define ROUTE_FINAL_SPEED_TOL_MM_S       (8.0f)
#define ROUTE_FINAL_STABLE_FRAMES        (15U)
/*
 * Startup feedforward: when the start button is pressed the camera may not
 * yet see the ball (static friction holds it in place).  This fixed angle
 * tilts the platform so the ball begins moving toward +5 cm even without
 * vision feedback.  The feedforward is cleared as soon as the camera
 * detects the ball, handing control back to the closed-loop controller.
 *
 * Negative angle drives the ball toward the positive (+5 cm) side.
 */
#define ROUTE_START_FEEDFORWARD_DEG       (-2.0f)

static BallRoute_State s_state;
static uint32_t s_lastFrameSequence;
static uint8_t s_stableFrames;

static float absoluteFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void resetConfirmation(void)
{
    s_stableFrames = 0U;
    s_lastFrameSequence = BallBalance_GetVisionFrameSequence();
}

void BallRoute_Init(void)
{
    s_state = BALL_ROUTE_IDLE;
    resetConfirmation();
}

void BallRoute_Start(void)
{
    /*
     * Skip the return-to-zero phase entirely.  Directly target +5 cm
     * and apply a fixed feedforward tilt so the ball starts moving even
     * when the camera cannot detect it at the start.
     */
    ZeroControl_SetTargetMm(ROUTE_POSITIVE_TARGET_MM);
    ZeroControl_SetFeedforwardAngleDeg(ROUTE_START_FEEDFORWARD_DEG);
    ZeroControl_SetEnabled(true);
    s_state = BALL_ROUTE_TO_POSITIVE;
    resetConfirmation();
}

void BallRoute_Stop(void)
{
    ZeroControl_SetEnabled(false);
    ZeroControl_SetFeedforwardAngleDeg(0.0f);
    ZeroControl_SetTargetMm(ROUTE_ZERO_TARGET_MM);
    s_state = BALL_ROUTE_IDLE;
    resetConfirmation();
}

void BallRoute_Toggle(void)
{
    if ((s_state == BALL_ROUTE_IDLE) || (s_state == BALL_ROUTE_DONE)) {
        BallRoute_Start();
    } else {
        BallRoute_Stop();
    }
}

void BallRoute_Update(void)
{
    uint32_t sequence;
    float positionMm;
    float rawPositionMm;
    float velocityMmS;
    bool conditionMet = false;

    if ((s_state == BALL_ROUTE_IDLE) || (s_state == BALL_ROUTE_DONE)) return;
    if (!BallBalance_IsBallVisible()) {
        s_stableFrames = 0U;
        return;
    }

    /*
     * Clear the startup feedforward as soon as the camera sees the ball.
     * The feedforward was only needed to break static friction at the
     * start; the closed-loop controller takes over from here.
     */
    if (ZeroControl_GetFeedforwardAngleDeg() != 0.0f) {
        ZeroControl_SetFeedforwardAngleDeg(0.0f);
    }

    sequence = BallBalance_GetVisionFrameSequence();
    if (sequence == s_lastFrameSequence) return;
    s_lastFrameSequence = sequence;
    positionMm = ZeroControl_GetPositionMm();
    rawPositionMm = BallBalance_GetVisionPositionMm();
    velocityMmS = ZeroControl_GetVelocityMmS();

    switch (s_state) {
    case BALL_ROUTE_WAIT_ZERO:
        conditionMet =
            (absoluteFloat(positionMm) <= ROUTE_ZERO_POSITION_TOL_MM) &&
            (absoluteFloat(velocityMmS) <= ROUTE_ZERO_SPEED_TOL_MM_S);
        if (conditionMet && (++s_stableFrames >= ROUTE_ZERO_STABLE_FRAMES)) {
            ZeroControl_SetTargetMm(ROUTE_POSITIVE_TARGET_MM);
            s_state = BALL_ROUTE_TO_POSITIVE;
            s_stableFrames = 0U;
        }
        break;

    case BALL_ROUTE_TO_POSITIVE:
        conditionMet = (rawPositionMm >= ROUTE_POSITIVE_SWITCH_MM);
        if (conditionMet) {
            ZeroControl_SetFeedforwardAngleDeg(0.0f);
            ZeroControl_SetTargetMm(ROUTE_NEGATIVE_TARGET_MM);
            s_state = BALL_ROUTE_TO_NEGATIVE;
            s_stableFrames = 0U;
        }
        break;

    case BALL_ROUTE_TO_NEGATIVE:
        conditionMet =
            (absoluteFloat(positionMm - ROUTE_NEGATIVE_TARGET_MM) <=
                ROUTE_FINAL_POSITION_TOL_MM) &&
            (absoluteFloat(velocityMmS) <= ROUTE_FINAL_SPEED_TOL_MM_S);
        if (conditionMet && (++s_stableFrames >= ROUTE_FINAL_STABLE_FRAMES)) {
            /* Keep the cascade enabled so the ball remains at -5 cm. */
            s_state = BALL_ROUTE_DONE;
            s_stableFrames = 0U;
        }
        break;

    default:
        break;
    }

    if (!conditionMet) s_stableFrames = 0U;
}

bool BallRoute_IsRunning(void)
{
    return (s_state != BALL_ROUTE_IDLE) && (s_state != BALL_ROUTE_DONE);
}

BallRoute_State BallRoute_GetState(void)
{
    return s_state;
}
