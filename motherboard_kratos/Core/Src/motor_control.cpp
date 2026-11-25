extern "C" {
    #include <rcl/rcl.h>
    #include <rclc/executor.h>
}
#include "motor_control.h"
#include "swerve_drive.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <stdbool.h>
volatile bool   flag = false;   // true = arm active (set from payload_cmd)
volatile double arm_cmd[7] = {0};
volatile double ld_cmd[10] = {0};

// =====================================================
// ROS 2 HOOKS (micro-ROS skeleton)
// =====================================================
extern "C" {
    bool motor_ros_init(rcl_node_t *node, rclc_executor_t *executor) { (void)node; (void)executor; return true; }
    void motor_ros_fini(rcl_node_t *node) { (void)node; }
    void motor_ros_publish(void) { }
}

extern "C" {

// =====================================================
// USEFUL DEFINITIONS FOR ARM CODE
// =====================================================
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;

typedef struct {

		TIM_HandleTypeDef *timer;
		uint32_t channel;
		GPIO_TypeDef *dir_port;
		uint16_t dir_pin;

	} ArmMotor;

//arm motor definition
ArmMotor arm_motors[5] =
	{
	    {&htim8, TIM_CHANNEL_1, GPIOD, GPIO_PIN_7},  // conn_set_1: PC6 (Bevel 1 PWM), PD7 (Bevel 1 DIR)
	    {&htim8, TIM_CHANNEL_2, GPIOB, GPIO_PIN_8},  // conn_set_1: PC7 (Bevel 2 PWM), PB8 (Bevel 2 DIR)
	    {&htim8, TIM_CHANNEL_3, GPIOB, GPIO_PIN_9},  // conn_set_2: PC8 (Shoulder PWM), PB9 (Shoulder DIR)
	    {&htim8, TIM_CHANNEL_4, GPIOD, GPIO_PIN_10}, // conn_set_2: PC9 (Elbow PWM), PD10 (Elbow DIR)
	    {&htim15, TIM_CHANNEL_1, GPIOD, GPIO_PIN_11}, // conn_set_3: PE5 (Base PWM), PD11 (Base DIR)
	};

typedef enum {
        BEVEL_1         = 0,  // conn_set_1: PC6, PD7
        BEVEL_2         = 1,  // conn_set_1: PC7, DIR x
        SHOULDER        = 2,  // conn_set_2: PC8, DIR x
        ELBOW           = 3,  // conn_set_2: PC9, PD10
        BASE            = 4,  // conn_set_3: PE5, PD11
        LINEAR_ACTUATOR = 5,  // conn_set_3: PC10, PC11
        GRIPPER         = 6,
        ARM_MOTOR_COUNT       // Automatically equals 7 (useful for array sizing / loops)
} ArmMotorID;

//set all the motors
void Arm_Motor_Init()
{
	for (int i = 0; i < 5; i++) {
		if (arm_motors[i].timer != nullptr) {
			HAL_TIM_PWM_Start(arm_motors[i].timer, arm_motors[i].channel);
		}
	}
	__HAL_TIM_MOE_ENABLE(&htim8);
	__HAL_TIM_MOE_ENABLE(&htim15);
}

//code to control one motor

void Arm_Set_Motor(uint8_t motor_id, int16_t input)
{

	uint16_t duty_cycle = (input > 0) ? input : -1*input;
	duty_cycle = (duty_cycle > 255) ? 255 : duty_cycle;

	//decide direction (negative means pin = 0 means backwards)
	if (arm_motors[motor_id].dir_port != nullptr) {
		HAL_GPIO_WritePin(arm_motors[motor_id].dir_port, arm_motors[motor_id].dir_pin, (input > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	if (arm_motors[motor_id].timer != nullptr) {
		uint32_t arr = __HAL_TIM_GET_AUTORELOAD(arm_motors[motor_id].timer);
		uint32_t compare_val = ((uint32_t)duty_cycle * arr) / 255;
		__HAL_TIM_SET_COMPARE(arm_motors[motor_id].timer, arm_motors[motor_id].channel, compare_val);
	}

}

//control all the arms
void Arm_Set_All(int16_t* input_array)
{
	for (int i =0; i < 5; i++)
	{
		Arm_Set_Motor(i, input_array[i]);
	}
}

// FreeRTOS Task Entry Point - called in a loop by StartTask02 (freertos.c).
void motor_control_Run(void *argument)
{
    (void)argument;

    static bool initialised = false;
    static uint32_t last_tick = 0;
    if (!initialised)
    {
        Swerve_Init();
        Arm_Motor_Init();
        last_tick = osKernelGetTickCount();
        initialised = true;
        return;   // first real update next cycle, with a measured dt
    }

    // Measure the real period instead of assuming the osDelay() value.
    uint32_t now = osKernelGetTickCount();
    float dt = (float)(now - last_tick) * 0.001f;   // 1 kHz tick
    last_tick = now;

    Swerve_Update(dt);   // 4 drive + 4 steer

    if (flag == 1)
    {
        // Snapshot pre-computed arm values from microros_main.c
        // (static: keeps 56 B off the 128-word motor_control stack)
        static double cmd[7];
        taskENTER_CRITICAL();
        for (int i = 0; i < 7; i++) {
            cmd[i] = arm_cmd[i];
        }
        taskEXIT_CRITICAL();

        // Pass directly to arm motors - zero math needed in motor_control!
        Arm_Set_Motor(BASE, (int16_t)cmd[0]);     // Base (conn_set_3: PE5, PD11)
        Arm_Set_Motor(SHOULDER, (int16_t)cmd[1]); // Shoulder (conn_set_2: PC8, PB9)
        Arm_Set_Motor(ELBOW, (int16_t)cmd[2]);    // Elbow (conn_set_2: PC9, PD10)
        Arm_Set_Motor(BEVEL_1, (int16_t)cmd[3]);  // Bevel 1 (conn_set_1: PC6, PD7)
        Arm_Set_Motor(BEVEL_2, (int16_t)cmd[4]);  // Bevel 2 (conn_set_1: PC7, PB8)

        // Linear Actuator (Cytron MD10A: PC10 PWM, PC11 DIR)
        if (cmd[5] > 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // Extend
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // PWM ON
        } else if (cmd[5] < 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // Retract
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // PWM ON
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // Stop
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
        }
    }
    else
    {
        Arm_Set_Motor(BASE, 0);
        Arm_Set_Motor(SHOULDER, 0);
        Arm_Set_Motor(ELBOW, 0);
        Arm_Set_Motor(BEVEL_1, 0);
        Arm_Set_Motor(BEVEL_2, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // Stop Linear Actuator
    }
}

} // end extern "C"
