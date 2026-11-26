/*
 * drive.cpp - 4-wheel independent-steer drive (4 drive + 4 steer motors).
 *
 * Drive motors : SpeedMotor (PWM + single DIR pin), quadrature encoder on TIM1..TIM4.
 * Steer motors : SteerMotor (PWM + single DIR pin), AS5600 absolute encoder on the
 *                output shaft; all four AS5600s share address 0x36, so they sit
 *                behind a PCA9548A I2C mux on I2C1 (PB6 SCL / PB7 SDA).
 *
 * Pin map follows the fabricated motherboard (Electronics_Motherboard, branch
 * NEW_MOTHERBOARD_FILES_dev). See the HARDWARE MAP below.
 */

#include "drive.h"
#include "SpeedMotor.h"
#include "SteerMotor.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cmath>

extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;   // drive encoders
extern TIM_HandleTypeDef htim5, htim13, htim14, htim15, htim16, htim17;  // PWM
extern I2C_HandleTypeDef hi2c1;

// =====================================================================
// TUNING / SAFETY CONSTANTS
// =====================================================================
#define CMD_TIMEOUT_MS          500     // no /drive_controller/rover for this long -> drive stops
                                        // (high level publishes at 50 Hz, so this only trips on link loss)

// Drive encoder: 13 PPR motor encoder, 181:1 gearbox. The x2 / x4 edge
// multiplier is read from each timer's encoder mode at init (TIM2 is TI12 = x4,
// TIM1/3/4 are TI1 = x2 in the .ioc).
#define DRIVE_ENC_PPR           13.0f
#define DRIVE_GEAR_RATIO        181.0f
#define SPEED_WINDOW            5       // speed = counts over the last N cycles (N x 10 ms)

// Steering error gate: while the wheels are still swinging to a new angle, the
// drive wheels would fight each other. Full speed below GATE_LO, zero above GATE_HI.
#define STEER_GATE_LO_DEG       10.0f
#define STEER_GATE_HI_DEG       30.0f

// Steering safety
#define ENC_FAIL_LIMIT          10      // consecutive failed AS5600 reads -> that steer motor off, drive off
#define STEER_OVERTRAVEL_DEG    15.0f   // measured angle outside [0-15, 180+15] -> latched fault
                                        // (catches a reversed STEER motorDir / encoderDir before it
                                        //  winds the wheel into the hard stop forever)

// Motor voltage clamp: the battery is above both motors' rated voltage, so PWM
// duty is capped at V_rated / V_supply. That makes the AVERAGE voltage at the
// motor equal its rating. Applied in the motor classes and as the PID limit.
#define SUPPLY_VOLTAGE          22.0f   // V, battery
#define DRIVE_MOTOR_RATED_V     18.0f   // V -> max duty 81.8 %
#define STEER_MOTOR_RATED_V     12.0f   // V -> max duty 54.5 %
#define DRIVE_MAX_DUTY          (100.0f * DRIVE_MOTOR_RATED_V / SUPPLY_VOLTAGE)
#define STEER_MAX_DUTY          (100.0f * STEER_MOTOR_RATED_V / SUPPLY_VOLTAGE)
static_assert(DRIVE_MOTOR_RATED_V <= SUPPLY_VOLTAGE && STEER_MOTOR_RATED_V <= SUPPLY_VOLTAGE,
              "rated voltage above supply: duty cap would exceed 100 %");

// PCA9548A mux + AS5600
#define PCA9548A_I2C_ADDR       (0x70 << 1)
#define AS5600_I2C_ADDR         (0x36 << 1)
#define AS5600_RAW_ANGLE_REG    0x0C
#define I2C_TIMEOUT_MS          2

// Default gains (can be changed live through g_* over SWD)
#define DRIVE_KP                10.0f   // from the previous 6-wheel firmware
#define DRIVE_KI                8.0f
#define DRIVE_KD                0.1f
#define STEER_KP                2.0f    // TODO: tune with pid_tuner_position.py (steer-tuner branch)
#define STEER_KI                0.0f
#define STEER_KD                0.05f

// =====================================================================
// HARDWARE MAP  (wheel order matches the high level: FL, FR, RL, RR)
// =====================================================================
//
//  One motherboard terminal per wheel: drive_conn_set_N (5-pin screw terminal)
//  pin 1 PWM / pin 2 DIR = drive motor, pin 3 PWM / pin 4 DIR = steer motor, pin 5 GND.
//
//  Wheel | Terminal | Drive PWM        DIR  | Drive encoder           | Steer PWM        DIR  | AS5600
//  ------+----------+-----------------------+-------------------------+-----------------------+---------------
//  FL    | J38 set1 | TIM5_CH1  PA0    PG2  | J25 TIM1 PE9 / PE11     | TIM5_CH2  PA1    PG3  | J17 mux ch0
//  FR    | J39 set2 | TIM5_CH3  PA2    PG4  | J26 TIM2 PA5 / PB3      | TIM15_CH2 PE6    PG5  | J16 mux ch1
//  RL    | J40 set3 | TIM16_CH1 PF6    PG6  | J27 TIM3 PA6 / PA7      | TIM17_CH1 PF7    PG7  | J15 mux ch2
//  RR    | J41 set4 | TIM13_CH1 PF8    PG8  | J28 TIM4 PD12 / PD13    | TIM14_CH1 PF9    PG9  | J14 mux ch3
//
//  ASSUMED (the board does not label it, verify on the rover): set N = wheel N in
//  FL, FR, RL, RR order; pins 1/2 = drive, 3/4 = steer; AB_enc_conn_N and
//  abs_enc_conn_N belong to wheel N. If a wheel is wired differently, only its
//  row in WHEEL_CFG needs to change.
//  Arm keeps TIM8_CH1..4 (PC6..PC9), TIM15_CH1 (PE5), PD7..PD11 - untouched.
//
//  Per-wheel calibration (all TODO on the real rover):
//   driveMotorDir : +1 if DIR pin HIGH drives the rover FORWARD (left/right sides
//                   are usually mirrored, so expect opposite signs per side)
//   driveEncDir   : +1 if the encoder counter goes UP when the rover drives FORWARD
//   steerHomeRaw  : AS5600 raw count with the wheel pointing straight (read g_steer_raw[i])
//   steerEncDir   : +1 if raw counts go UP as the steering angle goes UP (wheel turns right)
//   steerMotorDir : +1 if DIR HIGH moves the steering angle UP (use the tuner's dircheck)

struct WheelConfig {
    // drive
    TIM_HandleTypeDef* drivePwmTim;
    uint32_t           drivePwmCh;
    GPIO_TypeDef*      driveDirPort;
    uint16_t           driveDirPin;
    TIM_HandleTypeDef* driveEncTim;
    int8_t             driveMotorDir;
    int8_t             driveEncDir;
    // steer
    TIM_HandleTypeDef* steerPwmTim;
    uint32_t           steerPwmCh;
    GPIO_TypeDef*      steerDirPort;
    uint16_t           steerDirPin;
    uint8_t            steerMuxCh;
    uint16_t           steerHomeRaw;
    int8_t             steerEncDir;
    int8_t             steerMotorDir;
};

static const WheelConfig WHEEL_CFG[DRIVE_NUM_WHEELS] = {
    //  drive PWM              DIR                enc     mDir eDir | steer PWM              DIR                mux home  eDir mDir
    { &htim5,  TIM_CHANNEL_1, GPIOG, GPIO_PIN_2, &htim1, +1,  +1,   &htim5,  TIM_CHANNEL_2, GPIOG, GPIO_PIN_3, 0, 2048, +1,  +1 }, // FL  J38
    { &htim5,  TIM_CHANNEL_3, GPIOG, GPIO_PIN_4, &htim2, -1,  -1,   &htim15, TIM_CHANNEL_2, GPIOG, GPIO_PIN_5, 1, 2048, +1,  +1 }, // FR  J39
    { &htim16, TIM_CHANNEL_1, GPIOG, GPIO_PIN_6, &htim3, +1,  +1,   &htim17, TIM_CHANNEL_1, GPIOG, GPIO_PIN_7, 2, 2048, +1,  +1 }, // RL  J40
    { &htim13, TIM_CHANNEL_1, GPIOG, GPIO_PIN_8, &htim4, -1,  -1,   &htim14, TIM_CHANNEL_1, GPIOG, GPIO_PIN_9, 3, 2048, +1,  +1 }, // RR  J41
};
// pwmPeriod = 0 -> each motor reads its timer's ARR in begin() (65535 in the .ioc).
#define WHEEL_DRIVE(i) SpeedMotor(WHEEL_CFG[i].drivePwmTim, WHEEL_CFG[i].drivePwmCh, 0,           \
                                  WHEEL_CFG[i].driveDirPort, WHEEL_CFG[i].driveDirPin,            \
                                  WHEEL_CFG[i].driveDirPin, DRIVE_KP, DRIVE_KI, DRIVE_KD)
#define WHEEL_STEER(i) SteerMotor(WHEEL_CFG[i].steerPwmTim, WHEEL_CFG[i].steerPwmCh, 0,           \
                                  WHEEL_CFG[i].steerDirPort, WHEEL_CFG[i].steerDirPin,            \
                                  WHEEL_CFG[i].steerHomeRaw, WHEEL_CFG[i].steerEncDir,            \
                                  WHEEL_CFG[i].steerMotorDir, STEER_KP, STEER_KI, STEER_KD)

static SpeedMotor drive_motors[DRIVE_NUM_WHEELS] = {
    WHEEL_DRIVE(0), WHEEL_DRIVE(1), WHEEL_DRIVE(2), WHEEL_DRIVE(3)
};
static SteerMotor steer_motors[DRIVE_NUM_WHEELS] = {
    WHEEL_STEER(0), WHEEL_STEER(1), WHEEL_STEER(2), WHEEL_STEER(3)
};

// =====================================================================
// LIVE / DEBUG VARIABLES
// =====================================================================
extern "C" {
volatile float g_drive_kp = DRIVE_KP, g_drive_ki = DRIVE_KI, g_drive_kd = DRIVE_KD;
volatile float g_steer_kp = STEER_KP, g_steer_ki = STEER_KI, g_steer_kd = STEER_KD;

volatile float    g_drive_target_rpm[DRIVE_NUM_WHEELS] = {0};
volatile float    g_drive_rpm[DRIVE_NUM_WHEELS]        = {0};
volatile float    g_drive_duty[DRIVE_NUM_WHEELS]       = {0};
volatile float    g_steer_target_deg[DRIVE_NUM_WHEELS] = {90.0f, 90.0f, 90.0f, 90.0f};
volatile float    g_steer_deg[DRIVE_NUM_WHEELS]        = {90.0f, 90.0f, 90.0f, 90.0f};
volatile float    g_steer_duty[DRIVE_NUM_WHEELS]       = {0};
volatile uint32_t g_steer_raw[DRIVE_NUM_WHEELS]        = {0};
volatile uint32_t g_steer_enc_err[DRIVE_NUM_WHEELS]    = {0};
volatile uint32_t g_steer_enc_ok       = 0;
volatile uint32_t g_steer_fault        = 0;
volatile uint32_t g_drive_cmd_age_ms  = 0xFFFFFFFFu;
volatile float    g_drive_speed_scale = 0.0f;
}

// =====================================================================
// SHARED COMMAND (written by micro-ROS task, read by motor task)
// =====================================================================
struct DriveCmd {
    float    steer_deg[DRIVE_NUM_WHEELS];
    float    rpm[DRIVE_NUM_WHEELS];
    uint32_t stamp_ms;
    bool     valid;
};
static DriveCmd shared_cmd = {};

// =====================================================================
// INTERNAL STATE
// =====================================================================
static float    ticks_per_rev[DRIVE_NUM_WHEELS];
static uint16_t cnt_hist[DRIVE_NUM_WHEELS][SPEED_WINDOW];
static float    dt_hist[SPEED_WINDOW];
static uint8_t  hist_idx = 0;
static uint8_t  hist_filled = 0;

static uint8_t  enc_fail_count[DRIVE_NUM_WHEELS] = {0};
static bool     enc_have_reading[DRIVE_NUM_WHEELS] = {false};
static uint32_t overtravel_latched = 0;   // bit n = wheel n, cleared only by reset

// ---------------------------------------------------------------------
// Encoders
// ---------------------------------------------------------------------
static float EncoderEdgeMultiplier(TIM_HandleTypeDef* htim)
{
    uint32_t sms = htim->Instance->SMCR & TIM_SMCR_SMS;
    if (sms == (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1)) return 4.0f;  // TI12: both channels, both edges
    if (sms == TIM_SMCR_SMS_0 || sms == TIM_SMCR_SMS_1) return 2.0f; // TI1 or TI2 only
    return 4.0f;
}

// Select one mux channel and read that AS5600's raw angle (0..4095).
static bool ReadSteerEncoder(uint8_t ch, uint16_t* out)
{
    uint8_t sel = (uint8_t)(1u << ch);
    if (HAL_I2C_Master_Transmit(&hi2c1, PCA9548A_I2C_ADDR, &sel, 1, I2C_TIMEOUT_MS) != HAL_OK)
        return false;

    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, AS5600_RAW_ANGLE_REG, I2C_MEMADD_SIZE_8BIT,
                         buf, 2, I2C_TIMEOUT_MS) != HAL_OK)
        return false;

    *out = (uint16_t)((((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF);
    return true;
}

// Sweep all four AS5600s (~2.4 ms at the 100 kHz I2C1 timing in the .ioc).
// A failed channel keeps its last good value in g_steer_raw[].
static uint32_t ReadAllSteerEncoders(void)
{
    uint32_t ok = 0;
    for (uint8_t i = 0; i < DRIVE_NUM_WHEELS; i++) {
        uint16_t raw;
        if (ReadSteerEncoder(WHEEL_CFG[i].steerMuxCh, &raw)) {
            g_steer_raw[i] = raw;
            enc_have_reading[i] = true;
            enc_fail_count[i] = 0;
            ok |= (1u << i);
        } else {
            g_steer_enc_err[i]++;
            if (enc_fail_count[i] < ENC_FAIL_LIMIT) enc_fail_count[i]++;
        }
    }
    g_steer_enc_ok = ok;
    return ok;
}

// Measure all four wheel speeds (rover frame, rpm): counts moved since the
// oldest of the last SPEED_WINDOW samples, divided by the time in between.
// dt_hist[k] = time between sample k-1 and sample k.
static void ReadDriveSpeeds(float dt, float rpm_out[DRIVE_NUM_WHEELS])
{
    uint16_t now_cnt[DRIVE_NUM_WHEELS];
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++)
        now_cnt[i] = (uint16_t)__HAL_TIM_GET_COUNTER(WHEEL_CFG[i].driveEncTim);

    const uint8_t n = hist_filled;
    const uint8_t oldest = (n < SPEED_WINDOW) ? 0 : hist_idx;
    float span = dt;
    for (uint8_t k = 1; k < n; k++) span += dt_hist[(oldest + k) % SPEED_WINDOW];

    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        if (n == 0) { rpm_out[i] = 0.0f; continue; }
        int16_t delta = (int16_t)(now_cnt[i] - cnt_hist[i][oldest]);  // 16-bit wrap-safe
        float revs = (float)delta / ticks_per_rev[i];
        rpm_out[i] = (float)WHEEL_CFG[i].driveEncDir * (revs / span) * 60.0f;
    }

    // Push this sample (overwrites the oldest once the ring is full).
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) cnt_hist[i][hist_idx] = now_cnt[i];
    dt_hist[hist_idx] = dt;
    hist_idx = (uint8_t)((hist_idx + 1) % SPEED_WINDOW);
    if (hist_filled < SPEED_WINDOW) hist_filled++;
}

// Ensure a PWM timer has a 16-bit period. TIM5 is 32-bit; the .ioc sets its ARR
// to 65535, but CubeMX's default is 0xFFFFFFFF (0.015 Hz PWM), so keep this as a
// guard in case a regeneration resets it.
static void ClampPwmPeriod(TIM_HandleTypeDef* htim)
{
    if (__HAL_TIM_GET_AUTORELOAD(htim) > 0xFFFFu) {
        __HAL_TIM_SET_AUTORELOAD(htim, 0xFFFFu);
        __HAL_TIM_SET_COUNTER(htim, 0);
    }
}

static inline float Clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// =====================================================================
// PUBLIC API
// =====================================================================
extern "C" {

void Drive_Init(void)
{
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        ClampPwmPeriod(WHEEL_CFG[i].drivePwmTim);
        ClampPwmPeriod(WHEEL_CFG[i].steerPwmTim);
    }

    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        HAL_TIM_Encoder_Start(WHEEL_CFG[i].driveEncTim, TIM_CHANNEL_ALL);
        ticks_per_rev[i] = DRIVE_ENC_PPR * DRIVE_GEAR_RATIO *
                           EncoderEdgeMultiplier(WHEEL_CFG[i].driveEncTim);
        drive_motors[i].setMaxDuty(DRIVE_MAX_DUTY);
        steer_motors[i].setMaxDuty(STEER_MAX_DUTY);
        drive_motors[i].begin();
        steer_motors[i].begin();
    }

    // Hold every steering wheel where it is on boot (no jump before the first command).
    ReadAllSteerEncoders();
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        if (enc_have_reading[i]) {
            float a = steer_motors[i].rawToAngle((uint16_t)g_steer_raw[i]);
            steer_motors[i].setTargetAngle(a);
            g_steer_deg[i] = a;
        }
        g_steer_target_deg[i] = steer_motors[i].getTargetAngle();
    }
}

void Drive_SetCommand(const double steer_deg[DRIVE_NUM_WHEELS],
                       const double rpm[DRIVE_NUM_WHEELS])
{
    DriveCmd c;
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        // Reject the whole message on NaN/inf rather than drive on garbage.
        if (!std::isfinite(steer_deg[i]) || !std::isfinite(rpm[i])) return;
        c.steer_deg[i] = (float)steer_deg[i];
        c.rpm[i]       = (float)rpm[i];
    }
    c.stamp_ms = (uint32_t)xTaskGetTickCount();   // 1 kHz tick
    c.valid    = true;

    taskENTER_CRITICAL();
    shared_cmd = c;
    taskEXIT_CRITICAL();
}

void Drive_ClearCommand(void)
{
    taskENTER_CRITICAL();
    shared_cmd.valid = false;
    taskEXIT_CRITICAL();
}

void Drive_Stop(void)
{
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        drive_motors[i].stop();
        g_drive_target_rpm[i] = 0.0f;
        g_drive_duty[i] = 0.0f;
    }
}

void Drive_Update(float dt)
{
    if (dt <= 0.0f) return;

    // Large per-cycle buffers are static: only the motor_control task calls this,
    // and its stack is 128 words (set in the .ioc).
    static DriveCmd cmd;
    static float meas_rpm[DRIVE_NUM_WHEELS];

    // ---- 1. Snapshot the latest command ----
    taskENTER_CRITICAL();
    cmd = shared_cmd;
    taskEXIT_CRITICAL();

    uint32_t age = (uint32_t)xTaskGetTickCount() - cmd.stamp_ms;
    bool fresh = cmd.valid && age < CMD_TIMEOUT_MS;
    g_drive_cmd_age_ms = cmd.valid ? age : 0xFFFFFFFFu;

    // ---- 2. Sensors ----
    ReadAllSteerEncoders();
    ReadDriveSpeeds(dt, meas_rpm);

    // ---- 3. Steering ----
    uint32_t fault = 0;
    float max_err = 0.0f;
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        SteerMotor& s = steer_motors[i];
        s.setGains(g_steer_kp, g_steer_ki, g_steer_kd);
        if (fresh) s.setTargetAngle(cmd.steer_deg[i]);  // stale -> hold last target

        bool enc_lost = !enc_have_reading[i] || enc_fail_count[i] >= ENC_FAIL_LIMIT;
        if (!enc_lost) {
            float a = s.rawToAngle((uint16_t)g_steer_raw[i]);
            if (a < SteerMotor::MIN_ANGLE - STEER_OVERTRAVEL_DEG ||
                a > SteerMotor::MAX_ANGLE + STEER_OVERTRAVEL_DEG) {
                overtravel_latched |= (1u << i);
            }
        }

        if (enc_lost || (overtravel_latched & (1u << i))) {
            s.stop();
            fault |= (1u << i);
        } else {
            s.update((uint16_t)g_steer_raw[i], dt);
            float err = std::fabs(s.getTargetAngle() - s.getAngle());
            if (err > max_err) max_err = err;
        }

        g_steer_target_deg[i] = s.getTargetAngle();
        g_steer_deg[i]        = s.getAngle();
        g_steer_duty[i]       = s.getOutput();
    }
    g_steer_fault = fault;

    // ---- 4. Steering gate: don't drive hard while wheels are mid-swing ----
    float scale;
    if (fault != 0) {
        scale = 0.0f;   // never drive with a wheel we can't steer
    } else {
        scale = 1.0f - (max_err - STEER_GATE_LO_DEG) / (STEER_GATE_HI_DEG - STEER_GATE_LO_DEG);
        scale = Clampf(scale, 0.0f, 1.0f);
    }
    g_drive_speed_scale = scale;

    // ---- 5. Drive ----
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        g_drive_rpm[i] = meas_rpm[i];
    }
    if (!fresh) {
        Drive_Stop();   // link lost / no command yet: PWM off
        return;
    }
    for (int i = 0; i < DRIVE_NUM_WHEELS; i++) {
        SpeedMotor& d = drive_motors[i];
        const float dir = (float)WHEEL_CFG[i].driveMotorDir;
        float target = cmd.rpm[i] * scale;              // rover frame

        d.setGains(g_drive_kp, g_drive_ki, g_drive_kd);
        d.setTargetSpeed(dir * target);                 // motor frame: + = DIR high
        d.update(dir * meas_rpm[i], dt);

        g_drive_target_rpm[i] = target;
        g_drive_duty[i]       = d.getOutput();
    }
}

} // extern "C"
