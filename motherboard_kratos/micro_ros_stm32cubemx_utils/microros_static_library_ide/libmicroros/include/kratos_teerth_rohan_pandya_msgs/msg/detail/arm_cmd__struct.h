// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kratos_teerth_rohan_pandya_msgs:msg/ArmCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "kratos_teerth_rohan_pandya_msgs/msg/arm_cmd.h"


#ifndef KRATOS_TEERTH_ROHAN_PANDYA_MSGS__MSG__DETAIL__ARM_CMD__STRUCT_H_
#define KRATOS_TEERTH_ROHAN_PANDYA_MSGS__MSG__DETAIL__ARM_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/ArmCmd in the package kratos_teerth_rohan_pandya_msgs.
typedef struct kratos_teerth_rohan_pandya_msgs__msg__ArmCmd
{
  /// true = arm, false = life detection
  bool use_arm;
  double arm_cmd[7];
} kratos_teerth_rohan_pandya_msgs__msg__ArmCmd;

// Struct for a sequence of kratos_teerth_rohan_pandya_msgs__msg__ArmCmd.
typedef struct kratos_teerth_rohan_pandya_msgs__msg__ArmCmd__Sequence
{
  kratos_teerth_rohan_pandya_msgs__msg__ArmCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kratos_teerth_rohan_pandya_msgs__msg__ArmCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KRATOS_TEERTH_ROHAN_PANDYA_MSGS__MSG__DETAIL__ARM_CMD__STRUCT_H_
