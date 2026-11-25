#ifndef SWERVE_DRIVE_H_
#define SWERVE_DRIVE_H_

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
 * microros_main.c hands that array to Swerve_SetCommand(); the motor_control
 * task calls Swerve_Update() every cycle.
 */

#include <stdint.h>
#include <stdbool.h>

#define SWERVE_NUM_WHEELS 4

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3
} WheelId;

void Swerve_Init(void);

// One control cycle. dt_seconds = time since the previous call.
void Swerve_Update(float dt_seconds);

// Thread-safe; call from the micro-ROS callback.
// steer_deg[4]: 0..180 (90 = straight), rpm[4]: signed wheel speed.
void Swerve_SetCommand(const double steer_deg[SWERVE_NUM_WHEELS],
                       const double rpm[SWERVE_NUM_WHEELS]);

// Drop the current command (e.g. on micro-ROS agent disconnect): drive motors
// stop on the next cycle, steering holds its target. Thread-safe.
void Swerve_ClearCommand(void);

// Drive motors off now, steering holds its current target. Motor task only.
void Swerve_Stop(void);

// --- Live values for SWD / STM32CubeIDE Live Expressions ---
// Gains can be written live (e.g. by a pyOCD tuner); they are applied every cycle.
extern volatile float g_drive_kp, g_drive_ki, g_drive_kd;   // per rpm of error
extern volatile float g_steer_kp, g_steer_ki, g_steer_kd;   // per degree of error

extern volatile float    g_drive_target_rpm[SWERVE_NUM_WHEELS]; // after steer gating
extern volatile float    g_drive_rpm[SWERVE_NUM_WHEELS];        // measured, rover frame
extern volatile float    g_drive_duty[SWERVE_NUM_WHEELS];       // -100..100 %
extern volatile float    g_steer_target_deg[SWERVE_NUM_WHEELS];
extern volatile float    g_steer_deg[SWERVE_NUM_WHEELS];        // measured
extern volatile float    g_steer_duty[SWERVE_NUM_WHEELS];       // -100..100 %
extern volatile uint32_t g_steer_raw[SWERVE_NUM_WHEELS];        // AS5600 raw 0..4095 (use to calibrate home)
extern volatile uint32_t g_steer_enc_err[SWERVE_NUM_WHEELS];    // failed I2C reads since boot
extern volatile uint32_t g_steer_enc_ok;      // bit n = wheel n encoder read OK last cycle
extern volatile uint32_t g_steer_fault;       // bit n = wheel n steering disabled (encoder lost)
extern volatile uint32_t g_swerve_cmd_age_ms; // time since last /drive_controller/rover message
extern volatile float    g_swerve_speed_scale;// 0..1 drive scale from steering error gate

#ifdef __cplusplus
}
#endif

#endif /* SWERVE_DRIVE_H_ */
