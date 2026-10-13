#include "ball_balance/ball_balance.h"

#include <stdint.h>

#include "drivers/stepper_emm_v5.h"
#include "ti_msp_dl_config.h"
#include "drivers/timebase.h"

/* Hardware configuration. */
#define BALL_MOTOR_ADDR                    (2U)
#define BALL_DIR_LEFT_RAISE                (0U)
#define BALL_DIR_RIGHT_LOWER               (1U)
#define BALL_POSITION_RPM                  (300U)
#define BALL_ACCELERATION                  (50U)
#define BALL_PULSES_PER_REV                (12800.0f)
#define BALL_CONTROL_PERIOD_MS             (5U)
#define BALL_MAX_ANGLE_STEP_DEG            (0.20f)
#define BALL_REVERSE_STEP_DEG              (0.35f)
#define BALL_MAX_RIGHT_ANGLE_DEG           (30.0f)
#define BALL_MAX_LEFT_ANGLE_DEG            (30.0f)

/*
 * 第三问：车载过程中把钢球稳定在视觉中心。
 * 控制律为 angle = Kp * (position - target) + Kd * velocity。
 * 速度使用连续 3 个有效视觉帧估算，避免逐帧差分放大相机抖动。
 */
#define BALL_CENTER_KP_DEFAULT             (5.0f)
#define BALL_CENTER_KD_DEFAULT             (0.003f)
#define BALL_CENTER_ANGLE_LIMIT_DEG         (30.0f)
#define BALL_VELOCITY_FRAME_COUNT          (3U)

/*
 * MODE_BALL_LINE open-loop trajectory.
 *
 * Camera coordinates are left-positive and right-negative.  With the current
 * default mechanism direction, a positive motor angle sends the ball toward
 * A (-5 cm), and a negative motor angle sends it toward B (+5 cm).
 *
 * A is an instantaneous turning point: the A-braking tilt is retained after
 * the ball reaches zero speed, so the same command immediately accelerates it
 * toward B.  These times are initial calibration values; tune them from the
 * camera trace without feeding the camera position back into the controller.
 */
#define BALL_OPEN_A_TARGET_MM              (-50.0f)
#define BALL_OPEN_B_TARGET_MM              (50.0f)
#define BALL_OPEN_DRIVE_ANGLE_DEG          (9.0f)
#define BALL_OPEN_SOFT_BRAKE_ANGLE_DEG     (13.0f)
#define BALL_OPEN_A_ACCEL_MS               (700U)
#define BALL_OPEN_A_BRAKE_MS               (0U)
#define BALL_OPEN_B_ACCEL_MS               (225U)
#define BALL_OPEN_B_BRAKE_FAST_MS          (210U)
#define BALL_OPEN_B_BRAKE_SOFT_MS          (100U)
#define BALL_OPEN_LEVEL_SETTLE_MS          (300U)

/* Task targets: positive is camera-left, negative is camera-right. */
#define BALL_FIRST_TARGET_MM               (27.0f)
#define BALL_FINAL_TARGET_MM               (-5.0f)
#define BALL_FIRST_SWITCH_POSITION_MM      (8.0f)

/*
 * Position-to-speed outer loop.
 * v_limit = sqrt(2 * deceleration * remaining_distance)
 * gives a distance-dependent speed that naturally falls to zero.
 */
#define BALL_PLUS_MAX_SPEED_MM_S           (40.0f)
#define BALL_MINUS_MAX_SPEED_MM_S          (40.0f)
#define BALL_CENTER_MAX_SPEED_MM_S         (20.0f)
#define BALL_PLUS_STOPPING_DECEL_MM_S2     (18.0f)
#define BALL_MINUS_STOPPING_DECEL_MM_S2    (12.0f)
#define BALL_SPEED_ACCEL_MM_S2             (120.0f)
#define BALL_SPEED_DECEL_MM_S2             (220.0f)
#define BALL_POSITION_DEADBAND_MM          (2.0f)
#define BALL_NEAR_BRAKE_ZONE_MM            (20.0f)
#define BALL_PLUS_BRAKE_MAX_ANGLE_DEG      (2.2f)
#define BALL_MINUS_BRAKE_MAX_ANGLE_DEG     (2.5f)

/* Speed-to-angle inner control gains. */
#define BALL_POSITION_TRIM_KP              (0.025f)
#define BALL_PLUS_POSITION_GAIN_SCALE      (1.50f)
#define BALL_MINUS_POSITION_GAIN_SCALE     (1.50f)
#define BALL_POSITION_TRIM_KI              (0.0002f)
#define BALL_VELOCITY_KP                   (0.080f)
#define BALL_MINUS_VELOCITY_GAIN_SCALE     (1.50f)
#define BALL_CENTER_VELOCITY_GAIN_SCALE    (2.00f)
#define BALL_INTEGRAL_ACTIVE_MM            (20.0f)
#define BALL_INTEGRAL_ACTIVE_SPEED_MM_S    (15.0f)
#define BALL_INTEGRAL_LIMIT_MM_S           (100.0f)

/* Vision filtering and safety. */
#define BALL_START_DELAY_MS                (500U)
#define BALL_LOST_TIMEOUT_MS               (300U)
#define BALL_DEFAULT_FRAME_DT_S            (0.0167f)
#define BALL_MIN_FRAME_DT_S                (0.008f)
#define BALL_MAX_FRAME_DT_S                (0.100f)
#define BALL_POSITION_FILTER_ALPHA         (0.45f)
#define BALL_VELOCITY_FILTER_ALPHA         (0.30f)
#define BALL_MAX_MEASURED_SPEED_MM_S       (120.0f)
#define BALL_VISION_MAX_JUMP_PX            (45.0f)
#define BALL_VISION_JUMP_WINDOW_MS         (100U)

/*
 * Camera calibration.  The two sides use separate spans to compensate for
 * perspective.  Replace these constants with measured center-to-+5 and
 * center-to--5 pixel distances when final calibration data is available.
 */
#define CAMERA_CALIBRATION_HALF_MM         (50.0f)
#define CAMERA_POS5_SPAN_PX                (113.15f)
#define CAMERA_NEG5_SPAN_PX                (120.45f)
#define BALL_PATH_LEFT_OFFSET_MM           (10.0f)

/* Static-friction recovery, following the reference STM32 project. */
#define BALL_STUCK_SPEED_MM_S              (2.0f)
#define BALL_STUCK_MOVE_MM                 (2.0f)
#define BALL_STUCK_CONFIRM_MS              (150U)
#define BALL_PLUS_STUCK_START_DEG          (0.30f)
#define BALL_PLUS_STUCK_RAMP_DEG_S         (1.00f)
#define BALL_PLUS_STUCK_MAX_DEG            (1.20f)
#define BALL_MINUS_STUCK_START_DEG         (0.50f)
#define BALL_MINUS_STUCK_RAMP_DEG_S        (2.00f)
#define BALL_MINUS_STUCK_MAX_DEG           (3.00f)
#define BALL_MINUS_STATIC_MIN_ANGLE_DEG     (2.00f)
#define BALL_MINUS_LAUNCH_ANGLE_DEG         (4.00f)
#define BALL_MINUS_LAUNCH_TIME_MS           (180U)

typedef enum {
    BALL_STAGE_TO_PLUS_5 = 0,
    BALL_STAGE_TO_MINUS_5,
    BALL_STAGE_MANUAL
} BallStage;

typedef enum {
    BALL_OPEN_IDLE = 0,
    BALL_OPEN_TO_A_ACCEL,
    BALL_OPEN_TO_A_BRAKE,
    BALL_OPEN_TO_B_ACCEL,
    BALL_OPEN_TO_B_BRAKE_FAST,
    BALL_OPEN_TO_B_BRAKE_SOFT,
    BALL_OPEN_LEVEL,
    BALL_OPEN_DONE
} BallOpenStage;

static volatile float s_cameraCenterX;
static volatile float s_ballCenterX;
static volatile float s_positionMm;
static volatile uint32_t s_lastFrameMs;
static volatile uint32_t s_frameSequence;
static volatile bool s_frameValid;

static BallStage s_stage;
static bool s_enabled;
static bool s_motorRunning;
static bool s_filterInitialized;
static bool s_cameraZeroCaptured;
static bool s_relativePixelValid;
static bool s_lastTargetValid;
static bool s_stuckTracking;
static bool s_absoluteCenterMode;
static bool s_openLoopMode;

static float s_targetMm;
static float s_filteredPositionMm;
static float s_previousFilteredPositionMm;
static float s_filteredVelocityMmS;
static float s_referenceVelocityMmS;
static float s_positionIntegral;
static float s_cameraZeroMm;
static float s_previousRelativePixel;
static float s_stuckReferencePositionMm;
static float s_stuckBoostAngleDeg;
static float s_requestedAngleDeg;
static float s_motorAngleDeg;
static float s_outputAngleDeg;
static float s_positionKp;
static float s_positionKi;
static float s_velocityKp;
static float s_centerKp;
static float s_centerKd;
static float s_threeFrameVelocityMmS;
static float s_carFeedforwardDeg;

static uint32_t s_lastControlMs;
static uint32_t s_lastPidFrameMs;
static uint32_t s_processedFrameSequence;
static uint32_t s_startDelayEndMs;
static uint32_t s_stuckStartMs;
static uint32_t s_minusLaunchEndMs;
static uint32_t s_lastTargetPulses;
static uint8_t s_lastTargetDirection;
static BallOpenStage s_openStage;
static uint32_t s_openStageStartMs;

static float s_velocityHistoryPosition[BALL_VELOCITY_FRAME_COUNT];
static uint32_t s_velocityHistoryTimeMs[BALL_VELOCITY_FRAME_COUNT];
static uint8_t s_velocityHistoryCount;
static uint8_t s_velocityHistoryNext;

static volatile bool s_frameInMillimeters;
static volatile float s_directPositionMm;
static char s_rxFrame[80];
static uint8_t s_rxLength;

static float absoluteFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float moveToward(float current, float target, float maximumStep)
{
    if (current < target) {
        current += maximumStep;
        return (current > target) ? target : current;
    }
    if (current > target) {
        current -= maximumStep;
        return (current < target) ? target : current;
    }
    return current;
}

/* Newton iteration avoids adding a separate math-library dependency. */
static float positiveSqrt(float value)
{
    float result;
    uint8_t iteration;

    if (value <= 0.0f) return 0.0f;
    result = (value > 1.0f) ? value : 1.0f;
    for (iteration = 0U; iteration < 6U; iteration++) {
        result = 0.5f * (result + value / result);
    }
    return result;
}

static bool parseDecimal(const char *text, uint8_t length, float *valueOut)
{
    uint8_t index = 0U;
    uint8_t decimalDigits = 0U;
    bool decimalSeen = false;
    bool digitSeen = false;
    float value = 0.0f;
    float sign = 1.0f;
    float decimalScale = 0.1f;

    if (length == 0U) return false;
    if ((text[index] == '+') || (text[index] == '-')) {
        if (text[index] == '-') sign = -1.0f;
        index++;
    }

    for (; index < length; index++) {
        char ch = text[index];
        if ((ch >= '0') && (ch <= '9')) {
            digitSeen = true;
            if (!decimalSeen) {
                value = value * 10.0f + (float) (ch - '0');
            } else if (decimalDigits < 2U) {
                value += (float) (ch - '0') * decimalScale;
                decimalScale *= 0.1f;
                decimalDigits++;
            } else {
                return false;
            }
        } else if ((ch == '.') && !decimalSeen) {
            decimalSeen = true;
        } else {
            return false;
        }
    }

    if (!digitSeen) return false;
    *valueOut = sign * value;
    return true;
}

static void resetVelocityHistory(void)
{
    uint8_t index;

    for (index = 0U; index < BALL_VELOCITY_FRAME_COUNT; index++) {
        s_velocityHistoryPosition[index] = 0.0f;
        s_velocityHistoryTimeMs[index] = 0U;
    }
    s_velocityHistoryCount = 0U;
    s_velocityHistoryNext = 0U;
    s_threeFrameVelocityMmS = 0.0f;
}

static void pushVelocityHistory(float positionMm, uint32_t frameMs)
{
    uint8_t oldest;
    uint32_t elapsedMs;

    s_velocityHistoryPosition[s_velocityHistoryNext] = positionMm;
    s_velocityHistoryTimeMs[s_velocityHistoryNext] = frameMs;
    s_velocityHistoryNext = (uint8_t) (
        (s_velocityHistoryNext + 1U) % BALL_VELOCITY_FRAME_COUNT);

    if (s_velocityHistoryCount < BALL_VELOCITY_FRAME_COUNT) {
        s_velocityHistoryCount++;
    }
    if (s_velocityHistoryCount < BALL_VELOCITY_FRAME_COUNT) {
        s_threeFrameVelocityMmS = 0.0f;
        return;
    }

    oldest = s_velocityHistoryNext;
    elapsedMs = frameMs - s_velocityHistoryTimeMs[oldest];
    if ((elapsedMs < 20U) || (elapsedMs > 300U)) {
        s_threeFrameVelocityMmS = 0.0f;
        return;
    }
    s_threeFrameVelocityMmS =
        (positionMm - s_velocityHistoryPosition[oldest]) *
        1000.0f / (float) elapsedMs;
}

/*
 * MaixCAM2 protocols:
 *   Primary: #<position_cm>!    for example #-12.5! or #+3.25!
 *   Legacy : #<center_x>,<ball_x>!
 *   Lost   : #N!
 */
static void parseFrame(const char *frame, uint8_t length)
{
    float centerX;
    float ballX;
    float positionCm;
    uint8_t commaIndex = 0U;
    uint8_t index;

    if ((length < 3U) || (frame[0] != '#') ||
        (frame[length - 1U] != '!')) {
        return;
    }
    if ((length == 3U) && (frame[1] == 'N')) {
        s_frameValid = false;
        resetVelocityHistory();
        return;
    }

    for (index = 1U; index < (uint8_t) (length - 1U); index++) {
        if (frame[index] == ',') {
            commaIndex = index;
            break;
        }
    }
    if (commaIndex == 0U) {
        if (parseDecimal(&frame[1], (uint8_t) (length - 2U),
                         &positionCm) &&
            (positionCm >= -12.5f) && (positionCm <= 12.5f)) {
            /* The controller uses millimetres internally. */
            s_directPositionMm = positionCm * 10.0f;
            s_frameInMillimeters = true;
            s_lastFrameMs = Timebase_GetMillis();
            s_frameValid = true;
            s_frameSequence++;
        }
        return;
    }

    if ((commaIndex <= 1U) ||
        (commaIndex >= (uint8_t) (length - 2U))) {
        return;
    }

    if (parseDecimal(
            &frame[1], (uint8_t) (commaIndex - 1U), &centerX) &&
        parseDecimal(
            &frame[commaIndex + 1U],
            (uint8_t) (length - commaIndex - 2U), &ballX)) {
        s_cameraCenterX = centerX;
        s_ballCenterX = ballX;
        s_frameInMillimeters = false;
        s_lastFrameMs = Timebase_GetMillis();
        s_frameValid = true;
        s_frameSequence++;
    }
}

/*
 * K230 protocol used by the standalone third-question project:
 * BALL,<sequence>,<position_mm>,<valid>
 */
static void parseBallFrame(const char *frame, uint8_t length)
{
    uint8_t firstComma = 0U;
    uint8_t secondComma = 0U;
    uint8_t thirdComma = 0U;
    uint8_t index;
    float positionMm;

    if ((length < 10U) ||
        (frame[0] != 'B') || (frame[1] != 'A') ||
        (frame[2] != 'L') || (frame[3] != 'L') ||
        (frame[4] != ',')) {
        return;
    }

    for (index = 4U; index < length; index++) {
        if (frame[index] == ',') {
            if (firstComma == 0U) firstComma = index;
            else if (secondComma == 0U) secondComma = index;
            else {
                thirdComma = index;
                break;
            }
        }
    }
    if ((firstComma != 4U) || (secondComma == 0U) ||
        (thirdComma == 0U) || (thirdComma >= (uint8_t) (length - 1U))) {
        return;
    }

    if (frame[thirdComma + 1U] == '0') {
        s_frameValid = false;
        resetVelocityHistory();
        return;
    }
    if ((frame[thirdComma + 1U] != '1') ||
        !parseDecimal(
            &frame[secondComma + 1U],
            (uint8_t) (thirdComma - secondComma - 1U),
            &positionMm)) {
        return;
    }

    s_directPositionMm = positionMm;
    s_frameInMillimeters = true;
    s_lastFrameMs = Timebase_GetMillis();
    s_frameValid = true;
    s_frameSequence++;
}

static void receiveByte(uint8_t data)
{
    if (data == '#') {
        s_rxLength = 0U;
        s_rxFrame[s_rxLength++] = (char) data;
        return;
    }
    if ((data == 'B') && (s_rxLength == 0U)) {
        s_rxFrame[s_rxLength++] = (char) data;
        return;
    }
    if ((data == '\r') || (data == '\n')) {
        if ((s_rxLength > 0U) && (s_rxFrame[0] == 'B')) {
            parseBallFrame(s_rxFrame, s_rxLength);
        }
        s_rxLength = 0U;
        return;
    }
    if (s_rxLength == 0U) return;
    if (s_rxLength >= sizeof(s_rxFrame)) {
        s_rxLength = 0U;
        return;
    }

    s_rxFrame[s_rxLength++] = (char) data;
    if (data == '!') {
        parseFrame(s_rxFrame, s_rxLength);
        s_rxLength = 0U;
    }
}

static void resetStuckAssist(void)
{
    s_stuckTracking = false;
    s_stuckReferencePositionMm = 0.0f;
    s_stuckBoostAngleDeg = 0.0f;
    s_stuckStartMs = 0U;
}

static void clearControllerState(bool clearCamera)
{
    s_filteredPositionMm = 0.0f;
    s_previousFilteredPositionMm = 0.0f;
    s_filteredVelocityMmS = 0.0f;
    s_referenceVelocityMmS = 0.0f;
    s_positionIntegral = 0.0f;
    s_carFeedforwardDeg = 0.0f;
    s_requestedAngleDeg = 0.0f;
    s_outputAngleDeg = 0.0f;
    s_filterInitialized = false;
    s_relativePixelValid = false;
    s_lastTargetValid = false;
    s_previousRelativePixel = 0.0f;
    resetVelocityHistory();
    s_lastControlMs = Timebase_GetMillis();
    s_lastPidFrameMs = 0U;
    s_processedFrameSequence = s_frameSequence;
    s_startDelayEndMs = 0U;
    s_minusLaunchEndMs = 0U;
    resetStuckAssist();

    if (clearCamera) {
        s_cameraCenterX = 0.0f;
        s_ballCenterX = 0.0f;
        s_positionMm = 0.0f;
        s_cameraZeroMm = 0.0f;
        s_cameraZeroCaptured = false;
        s_lastFrameMs = 0U;
        s_frameSequence = 0U;
        s_processedFrameSequence = 0U;
        s_frameValid = false;
        s_frameInMillimeters = false;
        s_directPositionMm = 0.0f;
        s_rxLength = 0U;
    }
}

static void commandTargetAngle(float desiredAngle)
{
    float delta;
    float maximumStep;
    uint32_t targetPulses;
    uint8_t direction;

    desiredAngle = clampFloat(
        desiredAngle,
        -BALL_MAX_LEFT_ANGLE_DEG,
        BALL_MAX_RIGHT_ANGLE_DEG);

    delta = desiredAngle - s_motorAngleDeg;
    maximumStep = ((desiredAngle * s_motorAngleDeg) < 0.0f) ?
        BALL_REVERSE_STEP_DEG : BALL_MAX_ANGLE_STEP_DEG;
    s_motorAngleDeg = moveToward(
        s_motorAngleDeg, desiredAngle, maximumStep);

    direction = (s_motorAngleDeg >= 0.0f) ?
        BALL_DIR_LEFT_RAISE : BALL_DIR_RIGHT_LOWER;
    targetPulses = (uint32_t) (
        absoluteFloat(s_motorAngleDeg) *
        BALL_PULSES_PER_REV / 360.0f + 0.5f);
    s_outputAngleDeg = s_motorAngleDeg;

    if (s_lastTargetValid &&
        (targetPulses == s_lastTargetPulses) &&
        (direction == s_lastTargetDirection)) {
        return;
    }

    Emm_V5_Pos_Control(
        BALL_MOTOR_ADDR, direction, BALL_POSITION_RPM,
        BALL_ACCELERATION, targetPulses, 1U, false);
    s_lastTargetPulses = targetPulses;
    s_lastTargetDirection = direction;
    s_lastTargetValid = true;
    s_motorRunning = true;
}

/*
 * Open-loop phases send the absolute motor target immediately.  The Emm_V5
 * driver still applies its own RPM and acceleration limits.  Avoiding the
 * closed-loop controller's 0.2-degree software slew keeps the phase timing
 * deterministic when commanding the full +/-30-degree travel.
 */
static void commandOpenLoopAngle(float desiredAngle)
{
    uint32_t targetPulses;
    uint8_t direction;

    desiredAngle = clampFloat(
        desiredAngle,
        -BALL_MAX_LEFT_ANGLE_DEG,
        BALL_MAX_RIGHT_ANGLE_DEG);
    direction = (desiredAngle >= 0.0f) ?
        BALL_DIR_LEFT_RAISE : BALL_DIR_RIGHT_LOWER;
    targetPulses = (uint32_t) (
        absoluteFloat(desiredAngle) *
        BALL_PULSES_PER_REV / 360.0f + 0.5f);

    if (s_lastTargetValid &&
        (targetPulses == s_lastTargetPulses) &&
        (direction == s_lastTargetDirection)) {
        return;
    }

    Emm_V5_Pos_Control(
        BALL_MOTOR_ADDR, direction, BALL_POSITION_RPM,
        BALL_ACCELERATION, targetPulses, 1U, false);
    s_lastTargetPulses = targetPulses;
    s_lastTargetDirection = direction;
    s_lastTargetValid = true;
    s_motorAngleDeg = desiredAngle;
    s_outputAngleDeg = desiredAngle;
    s_motorRunning = true;
}

static void enterOpenStage(
    BallOpenStage stage, uint32_t now, float angleDeg)
{
    s_openStage = stage;
    s_openStageStartMs = now;
    commandOpenLoopAngle(angleDeg);
}

static void updateOpenLoopSequence(uint32_t now)
{
    uint32_t elapsedMs = now - s_openStageStartMs;

    switch (s_openStage) {
    case BALL_OPEN_IDLE:
        s_targetMm = BALL_OPEN_A_TARGET_MM;
        enterOpenStage(
            BALL_OPEN_TO_A_ACCEL, now,
            BALL_OPEN_DRIVE_ANGLE_DEG);
        break;

    case BALL_OPEN_TO_A_ACCEL:
        if (elapsedMs >= BALL_OPEN_A_ACCEL_MS) {
            enterOpenStage(
                BALL_OPEN_TO_A_BRAKE, now,
                -BALL_OPEN_DRIVE_ANGLE_DEG);
        }
        break;

    case BALL_OPEN_TO_A_BRAKE:
        if (elapsedMs >= BALL_OPEN_A_BRAKE_MS) {
            /*
             * At this instant the ball should have zero speed at A.  Keep the
             * same tilt: it now accelerates the ball toward B without a dwell.
             */
            s_targetMm = BALL_OPEN_B_TARGET_MM;
            s_openStage = BALL_OPEN_TO_B_ACCEL;
            s_openStageStartMs = now;
        }
        break;

    case BALL_OPEN_TO_B_ACCEL:
        if (elapsedMs >= BALL_OPEN_B_ACCEL_MS) {
            enterOpenStage(
                BALL_OPEN_TO_B_BRAKE_FAST, now,
                BALL_OPEN_DRIVE_ANGLE_DEG);
        }
        break;

    case BALL_OPEN_TO_B_BRAKE_FAST:
        if (elapsedMs >= BALL_OPEN_B_BRAKE_FAST_MS) {
            enterOpenStage(
                BALL_OPEN_TO_B_BRAKE_SOFT, now,
                BALL_OPEN_SOFT_BRAKE_ANGLE_DEG);
        }
        break;

    case BALL_OPEN_TO_B_BRAKE_SOFT:
        if (elapsedMs >= BALL_OPEN_B_BRAKE_SOFT_MS) {
            enterOpenStage(BALL_OPEN_LEVEL, now, 0.0f);
        }
        break;

    case BALL_OPEN_LEVEL:
        if (elapsedMs >= BALL_OPEN_LEVEL_SETTLE_MS) {
            s_openStage = BALL_OPEN_DONE;
            s_openStageStartMs = now;
            s_motorRunning = false;
        }
        break;

    case BALL_OPEN_DONE:
    default:
        /* Hold the absolute zero-angle target. */
        commandOpenLoopAngle(0.0f);
        break;
    }
}

static void stopMotor(void)
{
    clearControllerState(false);
    if (s_motorRunning) {
        Emm_V5_Stop_Now(BALL_MOTOR_ADDR, false);
        s_motorRunning = false;
    }
}

static void updateSequence(uint32_t now)
{
    if (s_stage != BALL_STAGE_TO_PLUS_5) return;

    /*
     * Do not wait for the ball to become stationary at +5.  Its inertia can
     * repeatedly carry it out of the acceptance band and prevent the old
     * stable-time condition from ever completing.  Entering the +5 band (or
     * passing it) is enough to start braking and travelling toward -5.
     */
    if (s_filteredPositionMm >= BALL_FIRST_SWITCH_POSITION_MM) {
        s_stage = BALL_STAGE_TO_MINUS_5;
        s_targetMm = BALL_FINAL_TARGET_MM;
        s_referenceVelocityMmS = 0.0f;
        s_positionIntegral = 0.0f;
        s_minusLaunchEndMs = now + BALL_MINUS_LAUNCH_TIME_MS;
        resetStuckAssist();
    }
}

static void updateCascadeControl(uint32_t now, float dtSeconds)
{
    float targetErrorMm;
    float distanceMm;
    float maximumSpeedMmS;
    float stoppingDecelMmS2;
    float stoppingSpeedMmS;
    float commandedSpeedMmS;
    float speedSlewMmS;
    float positionFeedbackErrorMm;
    float positionGain;
    float velocityErrorMmS;
    float desiredAngleDeg;
    float nearBrakeMaximumDeg;
    float stuckStartAngleDeg;
    float stuckRampDegS;
    float stuckMaximumAngleDeg;

    updateSequence(now);
    targetErrorMm = s_targetMm - s_filteredPositionMm;
    distanceMm = absoluteFloat(targetErrorMm);

    if (s_stage == BALL_STAGE_MANUAL) {
        /*
         * 第三问使用简单位置 PD。机构方向规定：位置为正时需要正电机
         * 角度把球推回中心，所以反馈使用 position-target 和正速度项。
         */
        desiredAngleDeg =
            s_centerKp * (s_filteredPositionMm - s_targetMm) +
            s_centerKd * s_threeFrameVelocityMmS +
            s_carFeedforwardDeg;
        s_positionIntegral = 0.0f;
        s_referenceVelocityMmS = 0.0f;
        s_requestedAngleDeg = clampFloat(
            desiredAngleDeg,
            -BALL_CENTER_ANGLE_LIMIT_DEG,
            BALL_CENTER_ANGLE_LIMIT_DEG);
        commandTargetAngle(s_requestedAngleDeg);
        return;
    }

    if (s_stage == BALL_STAGE_TO_PLUS_5) {
        maximumSpeedMmS = BALL_PLUS_MAX_SPEED_MM_S;
    } else if (s_stage == BALL_STAGE_TO_MINUS_5) {
        maximumSpeedMmS = BALL_MINUS_MAX_SPEED_MM_S;
    } else {
        maximumSpeedMmS = BALL_CENTER_MAX_SPEED_MM_S;
    }
    stoppingDecelMmS2 = (s_stage == BALL_STAGE_TO_PLUS_5) ?
        BALL_PLUS_STOPPING_DECEL_MM_S2 :
        BALL_MINUS_STOPPING_DECEL_MM_S2;

    if (distanceMm <= BALL_POSITION_DEADBAND_MM) {
        commandedSpeedMmS = 0.0f;
    } else {
        stoppingSpeedMmS = positiveSqrt(
            2.0f * stoppingDecelMmS2 * distanceMm);
        if (stoppingSpeedMmS > maximumSpeedMmS) {
            stoppingSpeedMmS = maximumSpeedMmS;
        }
        commandedSpeedMmS = (targetErrorMm >= 0.0f) ?
            stoppingSpeedMmS : -stoppingSpeedMmS;
    }

    /*
     * Acceleration and deceleration are independently limited.  This creates
     * the requested accelerate-then-decelerate speed curve without fixed
     * position thresholds or a one-frame direction jump.
     */
    if ((commandedSpeedMmS * s_referenceVelocityMmS < 0.0f) ||
        (absoluteFloat(commandedSpeedMmS) <
         absoluteFloat(s_referenceVelocityMmS))) {
        speedSlewMmS = BALL_SPEED_DECEL_MM_S2 * dtSeconds;
    } else {
        speedSlewMmS = BALL_SPEED_ACCEL_MM_S2 * dtSeconds;
    }
    s_referenceVelocityMmS = moveToward(
        s_referenceVelocityMmS, commandedSpeedMmS, speedSlewMmS);

    positionFeedbackErrorMm =
        s_filteredPositionMm - s_targetMm;
    positionGain = (s_stage == BALL_STAGE_TO_MINUS_5) ?
        s_positionKp * BALL_MINUS_POSITION_GAIN_SCALE :
        s_positionKp * BALL_PLUS_POSITION_GAIN_SCALE;
    velocityErrorMmS =
        s_filteredVelocityMmS - s_referenceVelocityMmS;

    if ((distanceMm <= BALL_INTEGRAL_ACTIVE_MM) &&
        (absoluteFloat(s_filteredVelocityMmS) <=
         BALL_INTEGRAL_ACTIVE_SPEED_MM_S)) {
        s_positionIntegral +=
            positionFeedbackErrorMm * dtSeconds;
        s_positionIntegral = clampFloat(
            s_positionIntegral,
            -BALL_INTEGRAL_LIMIT_MM_S,
            BALL_INTEGRAL_LIMIT_MM_S);
    } else {
        s_positionIntegral = 0.0f;
    }

    desiredAngleDeg =
        positionGain * positionFeedbackErrorMm +
        s_positionKi * s_positionIntegral +
        s_velocityKp * velocityErrorMmS;

    if (s_stage == BALL_STAGE_TO_MINUS_5) {
        desiredAngleDeg +=
            s_velocityKp *
            (BALL_MINUS_VELOCITY_GAIN_SCALE - 1.0f) *
            velocityErrorMmS;
    }

    /*
     * Immediately overcome static friction after changing from +5 to -5.
     * This floor is active only while the ball is nearly stationary and still
     * far from -5; motion feedback removes it as soon as the ball starts.
     */
    if ((s_stage == BALL_STAGE_TO_MINUS_5) &&
        (distanceMm > BALL_NEAR_BRAKE_ZONE_MM) &&
        (absoluteFloat(s_filteredVelocityMmS) <=
         BALL_STUCK_SPEED_MM_S) &&
        (desiredAngleDeg < BALL_MINUS_STATIC_MIN_ANGLE_DEG)) {
        desiredAngleDeg = BALL_MINUS_STATIC_MIN_ANGLE_DEG;
    }

    /*
     * A short, explicit rightward launch after +5 guarantees that static
     * friction cannot leave the ball parked at the first endpoint.
     */
    if ((s_stage == BALL_STAGE_TO_MINUS_5) &&
        ((int32_t) (s_minusLaunchEndMs - now) > 0) &&
        (desiredAngleDeg < BALL_MINUS_LAUNCH_ANGLE_DEG)) {
        desiredAngleDeg = BALL_MINUS_LAUNCH_ANGLE_DEG;
    }

    if (s_stage == BALL_STAGE_TO_PLUS_5) {
        stuckStartAngleDeg = BALL_PLUS_STUCK_START_DEG;
        stuckRampDegS = BALL_PLUS_STUCK_RAMP_DEG_S;
        stuckMaximumAngleDeg = BALL_PLUS_STUCK_MAX_DEG;
    } else {
        stuckStartAngleDeg = BALL_MINUS_STUCK_START_DEG;
        stuckRampDegS = BALL_MINUS_STUCK_RAMP_DEG_S;
        stuckMaximumAngleDeg = BALL_MINUS_STUCK_MAX_DEG;
    }

    if ((distanceMm > BALL_POSITION_DEADBAND_MM) &&
        (absoluteFloat(s_filteredVelocityMmS) <=
         BALL_STUCK_SPEED_MM_S)) {
        if (!s_stuckTracking) {
            s_stuckTracking = true;
            s_stuckReferencePositionMm = s_filteredPositionMm;
            s_stuckStartMs = now;
            s_stuckBoostAngleDeg = 0.0f;
        } else if (absoluteFloat(
                       s_filteredPositionMm -
                       s_stuckReferencePositionMm) >=
                   BALL_STUCK_MOVE_MM) {
            s_stuckReferencePositionMm = s_filteredPositionMm;
            s_stuckStartMs = now;
            s_stuckBoostAngleDeg = 0.0f;
        } else if ((uint32_t) (now - s_stuckStartMs) >=
                   BALL_STUCK_CONFIRM_MS) {
            if (s_stuckBoostAngleDeg <
                stuckStartAngleDeg) {
                s_stuckBoostAngleDeg =
                    stuckStartAngleDeg;
            } else {
                s_stuckBoostAngleDeg +=
                    stuckRampDegS * dtSeconds;
                if (s_stuckBoostAngleDeg >
                    stuckMaximumAngleDeg) {
                    s_stuckBoostAngleDeg =
                        stuckMaximumAngleDeg;
                }
            }
        }

        /* Positive target error means move left, which needs negative angle. */
        desiredAngleDeg += (targetErrorMm > 0.0f) ?
            -s_stuckBoostAngleDeg : s_stuckBoostAngleDeg;
    } else {
        resetStuckAssist();
    }

    /*
     * Near either endpoint, cap only the angle that opposes the current ball
     * velocity.  It can still brake, but cannot kick the ball hard enough to
     * launch it back across the target after stopping.
     */
    if (distanceMm <= BALL_NEAR_BRAKE_ZONE_MM) {
        nearBrakeMaximumDeg =
            (s_stage == BALL_STAGE_TO_PLUS_5) ?
            BALL_PLUS_BRAKE_MAX_ANGLE_DEG :
            BALL_MINUS_BRAKE_MAX_ANGLE_DEG;
        if ((s_filteredVelocityMmS > BALL_STUCK_SPEED_MM_S) &&
            (desiredAngleDeg >
             nearBrakeMaximumDeg)) {
            desiredAngleDeg =
                nearBrakeMaximumDeg;
        } else if ((s_filteredVelocityMmS <
                    -BALL_STUCK_SPEED_MM_S) &&
                   (desiredAngleDeg <
                    -nearBrakeMaximumDeg)) {
            desiredAngleDeg =
                -nearBrakeMaximumDeg;
        }
    }

    if ((distanceMm <= BALL_POSITION_DEADBAND_MM) &&
        (absoluteFloat(s_filteredVelocityMmS) <=
         BALL_STUCK_SPEED_MM_S)) {
        desiredAngleDeg = 0.0f;
        s_positionIntegral = 0.0f;
    }

    s_requestedAngleDeg = clampFloat(
        desiredAngleDeg,
        -BALL_MAX_LEFT_ANGLE_DEG,
        BALL_MAX_RIGHT_ANGLE_DEG);
    commandTargetAngle(s_requestedAngleDeg);
}

void BallBalance_ResetSequence(void)
{
    clearControllerState(true);
    s_absoluteCenterMode = false;
    s_openLoopMode = true;
    s_openStage = BALL_OPEN_IDLE;
    s_openStageStartMs = 0U;
    s_targetMm = BALL_OPEN_A_TARGET_MM;
}

void BallBalance_Init(void)
{
    s_positionKp = BALL_POSITION_TRIM_KP;
    s_positionKi = BALL_POSITION_TRIM_KI;
    s_velocityKp = BALL_VELOCITY_KP;
    s_centerKp = BALL_CENTER_KP_DEFAULT;
    s_centerKd = BALL_CENTER_KD_DEFAULT;
    s_enabled = false;
    s_motorRunning = false;
    s_motorAngleDeg = 0.0f;
    s_outputAngleDeg = 0.0f;
    s_absoluteCenterMode = false;
    s_openLoopMode = false;
    s_openStage = BALL_OPEN_IDLE;
    s_openStageStartMs = 0U;
    s_stage = BALL_STAGE_TO_PLUS_5;
    s_targetMm = BALL_FIRST_TARGET_MM;
    clearControllerState(true);

    /* Highest external IRQ priority: receive every MaixCAM2 frame promptly. */
    NVIC_SetPriority(UART_Camera_INST_INT_IRQN, 0U);
    NVIC_ClearPendingIRQ(UART_Camera_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_Camera_INST_INT_IRQN);
    Emm_V5_En_Control(BALL_MOTOR_ADDR, true, false);
}

void BallBalance_Update(void)
{
    uint32_t now = Timebase_GetMillis();
    uint32_t frameMs;
    uint32_t frameSequence;
    uint32_t frameDeltaMs;
    float relativePixel;
    float pixelJump;
    float rawPositionMm;
    float rawVelocityMmS;
    float dtSeconds;

    if ((uint32_t) (now - s_lastControlMs) <
        BALL_CONTROL_PERIOD_MS) {
        return;
    }
    s_lastControlMs = now;

    if (!s_enabled) {
        stopMotor();
        return;
    }

    if ((s_startDelayEndMs != 0U) &&
        ((int32_t) (s_startDelayEndMs - now) > 0)) {
        s_requestedAngleDeg = 0.0f;
        if (s_openLoopMode) {
            commandOpenLoopAngle(0.0f);
        } else {
            commandTargetAngle(0.0f);
        }
        return;
    }
    s_startDelayEndMs = 0U;

    if (s_openLoopMode) {
        updateOpenLoopSequence(now);
        return;
    }

    if (!s_frameValid ||
        ((uint32_t) (now - s_lastFrameMs) >
         BALL_LOST_TIMEOUT_MS)) {
        s_referenceVelocityMmS = 0.0f;
        s_positionIntegral = 0.0f;
        s_requestedAngleDeg = 0.0f;
        s_filterInitialized = false;
        s_relativePixelValid = false;
        resetStuckAssist();
        commandTargetAngle(0.0f);
        return;
    }

    frameSequence = s_frameSequence;
    if (frameSequence == s_processedFrameSequence) {
        commandTargetAngle(s_requestedAngleDeg);
        return;
    }
    s_processedFrameSequence = frameSequence;
    frameMs = s_lastFrameMs;

    relativePixel = s_cameraCenterX - s_ballCenterX;
    if (!s_frameInMillimeters && s_relativePixelValid) {
        frameDeltaMs = frameMs - s_lastPidFrameMs;
        pixelJump = absoluteFloat(
            relativePixel - s_previousRelativePixel);
        if ((frameDeltaMs <= BALL_VISION_JUMP_WINDOW_MS) &&
            (pixelJump > BALL_VISION_MAX_JUMP_PX)) {
            s_frameValid = false;
            return;
        }
    }
    if (s_frameInMillimeters) {
        rawPositionMm = s_directPositionMm;
    } else {
        s_previousRelativePixel = relativePixel;
        s_relativePixelValid = true;
        if (relativePixel >= 0.0f) {
            rawPositionMm =
                relativePixel * CAMERA_CALIBRATION_HALF_MM /
                CAMERA_POS5_SPAN_PX;
        } else {
            rawPositionMm =
                relativePixel * CAMERA_CALIBRATION_HALF_MM /
                CAMERA_NEG5_SPAN_PX;
        }
    }

    if (s_absoluteCenterMode) {
        /*
         * AB模式使用矩形框的真实中心：cameraCenterX-ballCenterX=0
         * 就是钢球中心，不套用直线任务的启动零点和赛道偏移。
         */
        s_cameraZeroMm = 0.0f;
        s_cameraZeroCaptured = true;
    } else {
        if (!s_cameraZeroCaptured) {
            s_cameraZeroMm = rawPositionMm;
            s_cameraZeroCaptured = true;
        }
        rawPositionMm -= s_cameraZeroMm;
        rawPositionMm -= BALL_PATH_LEFT_OFFSET_MM;
    }
    s_positionMm = rawPositionMm;

    if (!s_filterInitialized) {
        s_filteredPositionMm = rawPositionMm;
        s_previousFilteredPositionMm = rawPositionMm;
        s_filteredVelocityMmS = 0.0f;
        dtSeconds = BALL_DEFAULT_FRAME_DT_S;
        s_filterInitialized = true;
    } else {
        frameDeltaMs = frameMs - s_lastPidFrameMs;
        dtSeconds = (float) frameDeltaMs / 1000.0f;
        if ((dtSeconds < BALL_MIN_FRAME_DT_S) ||
            (dtSeconds > BALL_MAX_FRAME_DT_S)) {
            dtSeconds = BALL_DEFAULT_FRAME_DT_S;
        }

        s_filteredPositionMm += BALL_POSITION_FILTER_ALPHA *
            (rawPositionMm - s_filteredPositionMm);
        rawVelocityMmS =
            (s_filteredPositionMm -
             s_previousFilteredPositionMm) / dtSeconds;
        rawVelocityMmS = clampFloat(
            rawVelocityMmS,
            -BALL_MAX_MEASURED_SPEED_MM_S,
            BALL_MAX_MEASURED_SPEED_MM_S);
        s_filteredVelocityMmS += BALL_VELOCITY_FILTER_ALPHA *
            (rawVelocityMmS - s_filteredVelocityMmS);
        s_previousFilteredPositionMm = s_filteredPositionMm;
    }
    pushVelocityHistory(s_filteredPositionMm, frameMs);
    s_lastPidFrameMs = frameMs;

    updateCascadeControl(now, dtSeconds);
}

void BallBalance_SetEnabled(bool enabled)
{
    s_enabled = enabled;
    clearControllerState(true);

    if (enabled) {
        s_startDelayEndMs =
            Timebase_GetMillis() + BALL_START_DELAY_MS;
        Emm_V5_En_Control(BALL_MOTOR_ADDR, true, false);
    } else {
        stopMotor();
    }
}

void BallBalance_StartCenterControl(void)
{
    clearControllerState(true);
    s_absoluteCenterMode = true;
    s_openLoopMode = false;
    s_stage = BALL_STAGE_MANUAL;
    s_targetMm = 0.0f;
    s_enabled = true;
    s_startDelayEndMs = 0U;
    Emm_V5_En_Control(BALL_MOTOR_ADDR, true, false);
}

void BallBalance_SetCarFeedforwardDeg(float angleDeg)
{
    s_carFeedforwardDeg = clampFloat(angleDeg, -1.5f, 1.5f);
}

void BallBalance_SetTargetMm(float targetMm)
{
    clearControllerState(true);
    s_absoluteCenterMode = false;
    s_openLoopMode = false;
    s_stage = BALL_STAGE_MANUAL;
    s_targetMm = targetMm;
}

void BallBalance_SetPid(float kp, float ki, float kd)
{
    s_centerKp = kp;
    s_centerKd = kd;
    s_positionKi = ki;
    clearControllerState(false);
}

float BallBalance_GetPositionMm(void)
{
    return s_positionMm;
}

float BallBalance_GetTargetMm(void)
{
    return s_targetMm;
}

float BallBalance_GetOutputRpm(void)
{
    return s_outputAngleDeg;
}

float BallBalance_GetMotorAngleDeg(void)
{
    return s_motorAngleDeg;
}

bool BallBalance_IsBallVisible(void)
{
    return s_frameValid &&
        ((uint32_t) (Timebase_GetMillis() - s_lastFrameMs) <=
         BALL_LOST_TIMEOUT_MS);
}

bool BallBalance_IsFinalTarget(void)
{
    if (s_openLoopMode) {
        return s_openStage >= BALL_OPEN_TO_B_ACCEL;
    }
    return s_stage == BALL_STAGE_TO_MINUS_5;
}

bool BallBalance_IsEnabled(void)
{
    return s_enabled;
}

float BallBalance_GetVisionPositionMm(void)
{
    return s_directPositionMm;
}

uint32_t BallBalance_GetVisionFrameSequence(void)
{
    return s_frameSequence;
}

uint32_t BallBalance_GetVisionFrameMs(void)
{
    return s_lastFrameMs;
}

void UART_Camera_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_Camera_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_Camera_INST)) {
            receiveByte(DL_UART_Main_receiveData(UART_Camera_INST));
        }
        break;
    default:
        break;
    }
}
