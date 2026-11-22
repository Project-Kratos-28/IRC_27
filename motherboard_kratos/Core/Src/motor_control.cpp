extern "C" {
    #include <rcl/rcl.h>
    #include <rclc/executor.h>
}
#include "motor_control.h"
#include "SpeedMotor.h"
#include "main.h"
#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <stdbool.h>
volatile bool   flag = false;


double   motor_cmd_shared[6] = {0};
uint32_t motor_cmd_stamp_ms  = 0;
bool     motor_cmd_valid     = false;

// =====================================================
// ROS 2 INTEGRATION VARIABLES & HOOKS
// =====================================================
extern "C" {

    // Mandatory hooks required by the micro-ROS skeleton
    bool motor_ros_init(rcl_node_t *node, rclc_executor_t *executor) { (void)node; (void)executor; return true; }
    void motor_ros_fini(rcl_node_t *node) { (void)node; }
    void motor_ros_publish(void) { }
}

// =====================================================
// GLOBAL VARIABLES (Declared extern in header)
// =====================================================
volatile float g_target_rpm = 0.0f;
volatile float g_current_rpm = 0.0f;

// =====================================================
// HARDWARE PERIPHERAL DECLARATIONS
// =====================================================
// --- ENCODER TIMERS ---
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

// --- PWM TIMERS ---
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

// =====================================================
// MOTOR & HARDWARE ARRAYS
// =====================================================
static SpeedMotor drive_motors[6] = {
    SpeedMotor(&htim13, TIM_CHANNEL_1, 3199, GPIOG, GPIO_PIN_8, GPIO_PIN_8, 2.0f, 1.0f, 0.0f), // Motor 0
    SpeedMotor(&htim14, TIM_CHANNEL_1, 3199, GPIOG, GPIO_PIN_9, GPIO_PIN_9, 2.0f, 1.0f, 0.0f), // Motor 1
    SpeedMotor(&htim15, TIM_CHANNEL_2, 3199, GPIOG, GPIO_PIN_4, GPIO_PIN_4, 2.0f, 1.0f, 0.0f), // Motor 2
    SpeedMotor(&htim15, TIM_CHANNEL_1, 3199, GPIOD, GPIO_PIN_11, GPIO_PIN_11, 2.0f, 1.0f, 0.0f), // Motor 3
    SpeedMotor(&htim16, TIM_CHANNEL_1, 3199, GPIOG, GPIO_PIN_6, GPIO_PIN_6, 2.0f, 1.0f, 0.0f), // Motor 4
    SpeedMotor(&htim17, TIM_CHANNEL_1, 3199, GPIOG, GPIO_PIN_7, GPIO_PIN_7, 2.0f, 1.0f, 0.0f)  // Motor 5
};

static TIM_HandleTypeDef* drive_encoders[6] = {&htim1, &htim2, &htim3, &htim4, &htim5, &htim8};

static uint16_t drive_prev_counts[6] = {0};
static float current_rpms[6] = {0.0f};

// Live target speeds updated by ROS 2 (Initialized to 0.0f for safety)
static float target_rpms[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// =====================================================
// INTERNAL C++ FUNCTIONS
// =====================================================
static float ReadDriveSpeedRPM(uint8_t motor_id, float dt_seconds)
{
    if (dt_seconds <= 0.0f) return 0.0f;

    TIM_HandleTypeDef* htim = drive_encoders[motor_id];
    uint16_t current_count = (uint16_t)__HAL_TIM_GET_COUNTER(htim);

    int16_t delta_ticks = (int16_t)(current_count - drive_prev_counts[motor_id]);
    drive_prev_counts[motor_id] = current_count;

    const float ENCODER_PPR = 13.0f;
    const float GEAR_RATIO = 181.0f;
    const float ENCODER_MODE_MULT = 4.0f;
    const float TICKS_PER_REV = ENCODER_PPR * GEAR_RATIO * ENCODER_MODE_MULT;

    float revolutions = (float)delta_ticks / TICKS_PER_REV;
    return (revolutions / dt_seconds) * 60.0f;
}

// =====================================================
// C-ACCESSIBLE WRAPPER FUNCTIONS (Used by FreeRTOS)
// =====================================================
extern "C" {

void MotorControl_Init(void)
{
    float kp = 10.0f; // Set your tuned values here
    float ki = 8.0f;
    float kd = 0.1f;
    for (int i = 0; i < 6; i++) {
        drive_motors[i].begin();
        drive_motors[i].setGains(kp,ki,kd);
    }

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    for (int i = 0; i < 6; i++) {
        drive_prev_counts[i] = (uint16_t)__HAL_TIM_GET_COUNTER(drive_encoders[i]);
    }

}

void MotorControl_Update(float dt_seconds)
{

    if (dt_seconds <= 0.0f) return;

    // --- ROS 2 Integration & Safety Timeout ---
    double   cmd[6];
    uint32_t stamp;
    bool     valid;

    taskENTER_CRITICAL();
    memcpy(cmd, motor_cmd_shared, sizeof(cmd));
    stamp = motor_cmd_stamp_ms;
    valid = motor_cmd_valid;
    taskEXIT_CRITICAL();

    if (valid /*&& (osKernelGetTickCount() - stamp) < 500*/){ //commented out for the time being
        for (int i = 0; i < 6; i++) {
            target_rpms[i] = (float)cmd[i];
        }
    } else {
        // Connection lost or stale data -> Stop all motors
        for (int i = 0; i < 6; i++) {
            target_rpms[i] = 0.0f;
        }
    }
    // ------------------------------------------

    // 1. Read all 6 hardware timers (2 and 3 will read junk/zero)
        for (int i = 0; i < 6; i++) {
            current_rpms[i] = ReadDriveSpeedRPM(i, dt_seconds);
        }

        // 2. Fabricate the middle wheel values by averaging the sides
        current_rpms[4] = (current_rpms[0] + current_rpms[2]) / 2.0f; // Middle Right
        current_rpms[5] = (current_rpms[1] + current_rpms[3]) / 2.0f; // Middle Left

        // 3. Update the PID controllers
        for (int i = 0; i < 6; i++) {
            drive_motors[i].setTargetSpeed(target_rpms[i]);
            drive_motors[i].update(current_rpms[i], dt_seconds);
        }

    g_current_rpm = current_rpms[0];
}

// =====================================================
// USEFUL DEFINITIONS FOR ARM CODE
// =====================================================

typedef struct {

		TIM_HandleTypeDef *timer;
		uint32_t channel;
		GPIO_TypeDef *dir_port;
		uint16_t dir_pin;

	} ArmMotor;

//arm motor definition
ArmMotor arm_motors[6] =
	{
	    {&htim8, TIM_CHANNEL_1, GPIOD, GPIO_PIN_7}, //PC6
	    {&htim8, TIM_CHANNEL_2, GPIOD, GPIO_PIN_8}, //PC7
	    {&htim8, TIM_CHANNEL_3, GPIOD, GPIO_PIN_9}, //PC8
	    {&htim8, TIM_CHANNEL_4, GPIOD, GPIO_PIN_10}, //PC9
	    {&htim15, TIM_CHANNEL_1, GPIOD, GPIO_PIN_11}, //PE5
//		{, TIM_CHANNEL_, GPIOD, GPIO_PIN_}
	    // Sixth motor goes here
	};

//set all the motors
void Arm_Motor_Init()
{

	HAL_TIM_PWM_Start(arm_motors[0].timer, arm_motors[0].channel); //PC6
	HAL_TIM_PWM_Start(arm_motors[1].timer, arm_motors[1].channel); //PC7
	HAL_TIM_PWM_Start(arm_motors[2].timer, arm_motors[2].channel); //PC8
	HAL_TIM_PWM_Start(arm_motors[3].timer, arm_motors[3].channel); //PC9
	HAL_TIM_PWM_Start(arm_motors[4].timer, arm_motors[4].channel); //PE5

}

//code to control one motor

void Arm_Set_Motor(uint8_t motor_id, int16_t input)
{

	uint16_t duty_cycle = (input > 0) ? input : -1*input;
	duty_cycle = (duty_cycle > 255) ? 255 : duty_cycle;

	//decide direction (negative means pin = 0 means backwards)
	HAL_GPIO_WritePin(arm_motors[motor_id].dir_port, arm_motors[motor_id].dir_pin, (input > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(arm_motors[motor_id].timer, arm_motors[motor_id].channel, duty_cycle);

}

//control all the arms
void Arm_Set_All(int16_t* input_array)
{
	for (int i =0; i < 5; i++)
	{
		Arm_Set_Motor(i, input_array[i]);
	}
}

// FreeRTOS Task Entry Point
void motor_control_Run(void *argument)
{
    (void)argument;
    
    static bool initialised = false;
    if (!initialised)
    {
        MotorControl_Init();
        Arm_Motor_Init();
        initialised = true;
    }

    // freertos.c handles the osDelay(50) loop, so we pass 0.05s directly
    MotorControl_Update(0.05f); // Drive wheels

    if (flag == 1)
    {
        // Snapshot pre-computed arm values from microros_main.c
        double cmd[7];
        taskENTER_CRITICAL();
        for (int i = 0; i < 7; i++) {
            cmd[i] = arm_cmd[i];
        }
        taskEXIT_CRITICAL();

        // Pass directly to arm motors - zero math needed in motor_control!

        Arm_Set_Motor(0, (int16_t)cmd[0]); // Base
        Arm_Set_Motor(1, (int16_t)cmd[1]); // Shoulder
        Arm_Set_Motor(2, (int16_t)cmd[2]); // Elbow
        Arm_Set_Motor(3, (int16_t)cmd[3]); // Bevel 1 (w1 pre-computed in payload_cb)
        Arm_Set_Motor(4, (int16_t)cmd[4]); // Bevel 2 (w2 pre-computed in payload_cb)
//        Arm_Set_Motor(5, (int16_t)cmd[6]); // Linear actuator / Gripper (motors[5])
    }
    else
    {
        // Put LD code here
    }
}

} // end extern "C"
