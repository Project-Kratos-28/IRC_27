#include "microros_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* --- Micro-ROS & Standard Includes --- */
//#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "iwdg.h"
#include <rcutils/allocator.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <kratos_teerth_rohan_pandya_msgs/msg/arm_cmd.h>
#include <kratos_teerth_rohan_pandya_msgs/msg/wheel_cmd.h>
#include "motor_control.h"
#include "swerve_drive.h"
#include <stdbool.h>

/* --- Macros --- */
#define SPIN_TIMEOUT_MS   10
#define PING_PERIOD_MS    1000

/* Drive command from electronics_drive_high_level (drive_controller, branch dev):
 * std_msgs/Float64MultiArray, 8 values: [steer deg FL,FR,RL,RR | rpm FL,FR,RL,RR] */
#define DRIVE_CMD_TOPIC   "drive_controller/rover"
#define DRIVE_CMD_LEN     (2 * SWERVE_NUM_WHEELS)
extern IWDG_HandleTypeDef hiwdg1; 
#define WDG_FEED()        HAL_IWDG_Refresh(&hiwdg1)
#define RCCHECK(fn)  do { WDG_FEED(); if ((fn) != RCL_RET_OK) return false; } while (0)

/* --- Arm Polarity Macros (1 = Normal, -1 = Inverted) --- */
#define BASE_POLARITY             1
#define SHOULDER_POLARITY         1
#define ELBOW_POLARITY            1
#define PITCH_POLARITY            1
#define ROLL_POLARITY            -1

#define BEVEL_1_POLARITY          1
#define BEVEL_2_POLARITY          1
#define LINEAR_ACTUATOR_POLARITY  1
#define GRIPPER_POLARITY          1

/* --- Arm Max Speed Macros (0 to 255 PWM scale) --- */
#define BASE_MAX_SPEED            175
#define SHOULDER_MAX_SPEED        255
#define ELBOW_MAX_SPEED           255
#define BEVEL_1_MAX_SPEED         255
#define BEVEL_2_MAX_SPEED         255
#define GRIPPER_MAX_SPEED         255

/* --- Types --- */
typedef enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state_t;

/* --- Externs & Transport Prototypes --- */
extern UART_HandleTypeDef huart3;
extern volatile bool flag;

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

/* --- ROS Entities & Message Buffers --- */
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

rcl_subscription_t payload_message_subscriber;
rcl_subscription_t drive_message_subscriber;
rcl_subscription_t misc_message_subscriber;

kratos_teerth_rohan_pandya_msgs__msg__ArmCmd   arm_msg;
std_msgs__msg__Float64MultiArray               drive_msg;
static double                                  drive_msg_buf[DRIVE_CMD_LEN]; /* static storage, no malloc */
std_msgs__msg__Bool                            misc_msg;


std_msgs__msg__Bool misc_msg;


volatile HAL_StatusTypeDef tx_ret;   // global, so Live Expressions can read it
  // set to your number of joints/values

/* --- Subscriber Callbacks --- */
void payload_cb(const void * msgin) {
  const kratos_teerth_rohan_pandya_msgs__msg__ArmCmd * msg =
        (const kratos_teerth_rohan_pandya_msgs__msg__ArmCmd *)msgin;

  if (msg == NULL) return;

  // 1. Direct joint commands: Base (0), Shoulder (1), Elbow (2)
  double m0 = (msg->arm_cmd[0] * BASE_POLARITY * BASE_MAX_SPEED) / 255.0;
  double m1 = (msg->arm_cmd[1] * SHOULDER_POLARITY * SHOULDER_MAX_SPEED) / 255.0;
  double m2 = (msg->arm_cmd[2] * ELBOW_POLARITY * ELBOW_MAX_SPEED) / 255.0;

  // 2. Bevel differential mixing for wrist: Pitch (3), Roll (4)
  int32_t pitch = (int32_t)msg->arm_cmd[3] * PITCH_POLARITY;
  int32_t roll  = (int32_t)msg->arm_cmd[4] * ROLL_POLARITY;

  int32_t w1 = (pitch - roll) * BEVEL_1_POLARITY;
  int32_t w2 = (pitch + roll) * BEVEL_2_POLARITY;

  // Proportional normalization if combined magnitude exceeds 255
  int32_t abs_w1 = (w1 < 0) ? -w1 : w1;
  int32_t abs_w2 = (w2 < 0) ? -w2 : w2;
  int32_t max_mag = (abs_w1 > abs_w2) ? abs_w1 : abs_w2;

  if (max_mag > 255) {
    w1 = (w1 * 255) / max_mag;
    w2 = (w2 * 255) / max_mag;
  }

  w1 = (w1 * BEVEL_1_MAX_SPEED) / 255;
  w2 = (w2 * BEVEL_2_MAX_SPEED) / 255;

  // 3. Linear Actuator (5) and Gripper (6)
  double m5 = msg->arm_cmd[5] * LINEAR_ACTUATOR_POLARITY;
  double m6 = (msg->arm_cmd[6] * GRIPPER_POLARITY * GRIPPER_MAX_SPEED) / 255.0;

  // 4. Atomically store pre-computed motor values into shared arm_cmd
  taskENTER_CRITICAL();
  flag = msg->use_arm;
  arm_cmd[0] = m0;          // Base
  arm_cmd[1] = m1;          // Shoulder
  arm_cmd[2] = m2;          // Elbow
  arm_cmd[3] = (double)w1;  // Pre-mixed Bevel 1
  arm_cmd[4] = (double)w2;  // Pre-mixed Bevel 2
  arm_cmd[5] = m5;          // Linear Actuator
  arm_cmd[6] = m6;          // Gripper
  // Note: ld_cmd[10] from Test requires regenerating libmicroros with ArmCmd.ld_cmd
  taskEXIT_CRITICAL();
}

void drive_cb(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * m = (const std_msgs__msg__Float64MultiArray *)msgin;
  if (m == NULL || m->data.size < DRIVE_CMD_LEN) return;   /* malformed: ignore, timeout will stop us */

  /* data[0..3] steering deg, data[4..7] wheel rpm (wheel order FL, FR, RL, RR) */
  Swerve_SetCommand(&m->data.data[0], &m->data.data[SWERVE_NUM_WHEELS]);
}

void misc_cb(const void * msgin) {
  (void)msgin;
  //  for array
  // Add your misc callback logic here

}

/* --- Lifecycle Functions --- */
static bool ros_create(void)
{
  memset(&support, 0, sizeof(support));
  node = rcl_get_zero_initialized_node();
  payload_message_subscriber   = rcl_get_zero_initialized_subscription();
  drive_message_subscriber = rcl_get_zero_initialized_subscription();
  misc_message_subscriber  = rcl_get_zero_initialized_subscription();
  executor = rclc_executor_get_zero_initialized_executor();
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "rover_node", "", &support));

  RCCHECK(rclc_subscription_init_best_effort(&payload_message_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(kratos_teerth_rohan_pandya_msgs, msg, ArmCmd), "payload_cmd"));
  RCCHECK(rclc_subscription_init_best_effort(&drive_message_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), DRIVE_CMD_TOPIC));
  RCCHECK(rclc_subscription_init_best_effort(&misc_message_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "misc_topic"));

  /* Float64MultiArray has unbounded sequences: point them at static storage.
   * layout.dim gets capacity 0, so senders must leave the layout empty (the
   * high-level drive_controller does). */
  memset(&drive_msg, 0, sizeof(drive_msg));
  drive_msg.data.data     = drive_msg_buf;
  drive_msg.data.size     = 0;
  drive_msg.data.capacity = DRIVE_CMD_LEN;

  /* 3 handles for the 3 subscribers */
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &payload_message_subscriber, &arm_msg,   &payload_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &drive_message_subscriber,   &drive_msg, &drive_cb,   ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &misc_message_subscriber,    &misc_msg,  &misc_cb,    ON_NEW_DATA));
  return true;
}

static void ros_destroy(void)
{
  if (!rcl_context_is_valid(&support.context)) return;

  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_ret_t rc;
  rc = rclc_executor_fini(&executor);
  rc = rcl_subscription_fini(&payload_message_subscriber, &node);
  rc = rcl_subscription_fini(&drive_message_subscriber, &node);
  rc = rcl_subscription_fini(&misc_message_subscriber, &node);
  rc = rcl_node_fini(&node);
  rc = rclc_support_fini(&support);
  (void)rc; // Silence unused result warnings
}

/* --- Main Task Loop --- */
/* static buffers for the array message so the executor never mallocs */


void microros_main_Run(void *argument)
{
	      // add to Live Expressions
//for debugging uart
//	for (;;) {
//	    WDG_FEED();
//	    tx_ret = HAL_UART_Transmit(&huart3, (uint8_t*)"hello\r\n", 7, 100);
//	    osDelay(500);
//	}
	WDG_FEED();
  (void)argument;
  /* 2. transport */
  rmw_uros_set_custom_transport(true, (void *)&huart3,
            cubemx_transport_open, cubemx_transport_close,
            cubemx_transport_write, cubemx_transport_read);

//uncomment if no refresh
//  if ( rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//      for (;;) { osDelay(100); }   // or Error_Handler()
//    }

  /* 1. allocators FIRST, before any rcl/rmw call */
  rcl_allocator_t fa = rcutils_get_zero_initialized_allocator();
   fa.allocate      = microros_allocate;
   fa.deallocate    = microros_deallocate;
   fa.reallocate    = microros_reallocate;
   fa.zero_allocate = microros_zero_allocate;
   if (!rcutils_set_default_allocator(&fa)) {
     for (;;) {  osDelay(1000); }
   }



  state_t agent_state = WAITING_AGENT;
    uint32_t last_ping  = osKernelGetTickCount();
    int missed_spins    = 0;

  /* 3. static message storage (Float32MultiArray needs it) */




  for (;;)
  {
    switch (agent_state)
    {
    case WAITING_AGENT:
            agent_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                          ? AGENT_AVAILABLE : WAITING_AGENT;
            break;

      case AGENT_AVAILABLE:
        if (ros_create()) {
          last_ping    = osKernelGetTickCount();
          missed_spins = 0;
          agent_state  = AGENT_CONNECTED;
        } else {
          ros_destroy();
          agent_state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED:
      {
        uint32_t now = osKernelGetTickCount();
        if ((uint32_t)(now - last_ping) >= PING_PERIOD_MS) {
          last_ping = now;
          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            agent_state = AGENT_DISCONNECTED;
            break;
          }
        }
        rcl_ret_t r = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
        if (r != RCL_RET_OK && r != RCL_RET_TIMEOUT) {
          if (++missed_spins > 20) { agent_state = AGENT_DISCONNECTED; }
        } else {
          missed_spins = 0;
        }
        break;
      }

      case AGENT_DISCONNECTED:
        ros_destroy();
        taskENTER_CRITICAL();
        Swerve_ClearCommand();   /* drive stops next cycle; steering holds */
        flag = false;
        memset((void *)arm_cmd, 0, sizeof(arm_cmd));
        memset((void *)ld_cmd,  0, sizeof(ld_cmd));
        taskEXIT_CRITICAL();
        agent_state = WAITING_AGENT;
        break;
    }
    WDG_FEED();
    osDelay(1);
  }
}

//backup code if smtg breaks


//void microros_main_Run(void *argument)
//{
//  (void)argument;
//
//  rmw_uros_set_custom_transport(true, (void *)&huart3,
//      cubemx_transport_open, cubemx_transport_close,
//      cubemx_transport_write, cubemx_transport_read);
//
//  for(;;)
//   {
//       HAL_IWDG_Refresh(&hiwdg1);
// 	  rcl_allocator_t allocator = rcl_get_default_allocator();
// 	  rclc_support_t support;
// 	  rclc_support_init(&support, 0, NULL, &allocator);
//
//       // 1. Wait until the laptop agent is actually listening
//       while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
// //          HAL_IWDG_Refresh(&hiwdg);
//           osDelay(100);
//       }
//       ros_create();
//       rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//       for(;;)
//             {
//                 while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//
//
//               	  osDelay(500);
//
//                 }
//                 HAL_IWDG_Refresh(&hiwdg1);
//                 osDelay(1);
//             }
//   }
//
//  rcl_allocator_t fa = rcutils_get_zero_initialized_allocator();
//  fa.allocate      = microros_allocate;
//  fa.deallocate    = microros_deallocate;
//  fa.reallocate    = microros_reallocate;
//  fa.zero_allocate = microros_zero_allocate;
//  if (!rcutils_set_default_allocator(&fa)) {
//    for (;;) { osDelay(1000); }
//  }
//
//  state_t agent_state = WAITING_AGENT;
//  uint32_t last_ping  = osKernelGetTickCount();
//  int missed_spins    = 0;
//
//  for (;;)
//  {
//    switch (agent_state)
//    {
//      case WAITING_AGENT:
//        agent_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
//        break;
//
//      case AGENT_AVAILABLE:
//        if (ros_create()) {
//          last_ping    = osKernelGetTickCount();
//          missed_spins = 0;
//          agent_state  = AGENT_CONNECTED;
//        } else {
//          ros_destroy();
//          agent_state = WAITING_AGENT;
//        }
//        break;
//
//      case AGENT_CONNECTED:
//      {
//        uint32_t now = osKernelGetTickCount();
//        if ((uint32_t)(now - last_ping) >= PING_PERIOD_MS) {
//          last_ping = now;
//          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//            agent_state = AGENT_DISCONNECTED;
//            break;
//          }
//        }
//        rcl_ret_t r = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
//        if (r != RCL_RET_OK && r != RCL_RET_TIMEOUT) {
//          if (++missed_spins > 20) {
//            agent_state = AGENT_DISCONNECTED;
//            break;
//          }
//        } else {
//          missed_spins = 0;
//        }
//        break;
//      }
//
//      case AGENT_DISCONNECTED:
//        ros_destroy();
//        agent_state = WAITING_AGENT;
//        break;
//    }
//    WDG_FEED();
//    osDelay(1);
//  }
//}
