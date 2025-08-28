// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rospi_pre:srv/ChassisManager.idl
// generated code does not contain a copyright notice

#ifndef ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__STRUCT_H_
#define ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ChassisManager in the package rospi_pre.
typedef struct rospi_pre__srv__ChassisManager_Request
{
  rosidl_runtime_c__String command;
  uint8_t move_type;
} rospi_pre__srv__ChassisManager_Request;

// Struct for a sequence of rospi_pre__srv__ChassisManager_Request.
typedef struct rospi_pre__srv__ChassisManager_Request__Sequence
{
  rospi_pre__srv__ChassisManager_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rospi_pre__srv__ChassisManager_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ChassisManager in the package rospi_pre.
typedef struct rospi_pre__srv__ChassisManager_Response
{
  bool valid;
} rospi_pre__srv__ChassisManager_Response;

// Struct for a sequence of rospi_pre__srv__ChassisManager_Response.
typedef struct rospi_pre__srv__ChassisManager_Response__Sequence
{
  rospi_pre__srv__ChassisManager_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rospi_pre__srv__ChassisManager_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__STRUCT_H_
