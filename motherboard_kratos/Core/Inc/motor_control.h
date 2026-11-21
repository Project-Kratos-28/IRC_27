#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
extern double   motor_cmd_shared[6];
extern uint32_t motor_cmd_stamp_ms;
extern bool     motor_cmd_valid;
extern volatile bool   flag;
#ifndef ARM_WHEEL_VARS_DEFINED
#define ARM_WHEEL_VARS_DEFINED

extern volatile double arm_cmd[7];
extern volatile double wheel_info[8];
extern volatile double ld_cmd[10];
#endif
void motor_control_Run(void *argument);

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_CONTROL_H */
