#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool flag;            // true = arm commands active (payload_cmd.use_arm)

#ifndef ARM_WHEEL_VARS_DEFINED
#define ARM_WHEEL_VARS_DEFINED
extern volatile double arm_cmd[7];    // pre-mixed arm values from microros_main.c
extern volatile double ld_cmd[10];    // life-detection commands
#endif

// motor_control task body (drive + steer + arm). Called repeatedly by StartTask02.
void motor_control_Run(void *argument);

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_CONTROL_H */
