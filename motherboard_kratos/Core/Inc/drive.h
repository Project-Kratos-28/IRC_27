#ifndef DRIVE_H_
#define DRIVE_H_

/*
 * 4-wheel independent-steer drive (4 drive motors + 4 steering motors).
 *
 * Command source: the high-level drive_controller (electronics_drive_high_level,
 * branch dev) publishes ONE std_msgs/Float64MultiArray on /drive_controller/rover:
 *
 *     data[0..3]  steering angle, deg, wheel order FL, FR, RL, RR
 *                 0..180, 90 = straight ahead, > 90 = wheel turned right
 *     data[4..7]  wheel speed, rpm, wheel order FL, FR, RL, RR
 *                 + = rover moving forward
 *
 * microros_main.c hands that array to Drive_SetCommand(); the motor_control
 * task calls Drive_Update() every cycle.
 */

#include <stdint.h>
#include <stdbool.h>

#define DRIVE_NUM_WHEELS 4

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3
} WheelId;

void Drive_Init(void);

// One control cycle. dt_seconds = time since the previous call.
void Drive_Update(float dt_seconds);

// Thread-safe; call from the micro-ROS callback.
// steer_deg[4]: 0..180 (90 = straight), rpm[4]: signed wheel speed.
void Drive_SetCommand(const double steer_deg[DRIVE_NUM_WHEELS],
                       const double rpm[DRIVE_NUM_WHEELS]);

// Drop the current command (e.g. on micro-ROS agent disconnect): drive motors
// stop on the next cycle, steering holds its target. Thread-safe.
void Drive_ClearCommand(void);

// Drive motors off now, steering holds its current target. Motor task only.
void Drive_Stop(void);

// --- Live values for SWD / STM32CubeIDE Live Expressions ---
// Gains can be written live (e.g. by a pyOCD tuner); they are applied every cycle.
extern volatile float g_drive_kp, g_drive_ki, g_drive_kd;   // per rpm of error
extern volatile float g_steer_kp, g_steer_ki, g_steer_kd;   // per degree of error

extern volatile float    g_drive_target_rpm[DRIVE_NUM_WHEELS]; // after steer gating
extern volatile float    g_drive_rpm[DRIVE_NUM_WHEELS];        // measured, rover frame
extern volatile float    g_drive_duty[DRIVE_NUM_WHEELS];       // % , capped at +/-81.8 (18 V of 22 V)
extern volatile float    g_steer_target_deg[DRIVE_NUM_WHEELS];
extern volatile float    g_steer_deg[DRIVE_NUM_WHEELS];        // measured
extern volatile float    g_steer_duty[DRIVE_NUM_WHEELS];       // % , capped at +/-54.5 (12 V of 22 V)
extern volatile uint32_t g_steer_raw[DRIVE_NUM_WHEELS];        // AS5600 raw 0..4095 (use to calibrate home)
extern volatile uint32_t g_steer_enc_err[DRIVE_NUM_WHEELS];    // failed I2C reads since boot
extern volatile uint32_t g_steer_enc_ok;      // bit n = wheel n encoder read OK last cycle
extern volatile uint32_t g_steer_fault;       // bit n = wheel n steering disabled (encoder lost)
extern volatile uint32_t g_drive_cmd_age_ms; // time since last /drive_controller/rover message
extern volatile float    g_drive_speed_scale;// 0..1 drive scale from steering error gate

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H_ */
