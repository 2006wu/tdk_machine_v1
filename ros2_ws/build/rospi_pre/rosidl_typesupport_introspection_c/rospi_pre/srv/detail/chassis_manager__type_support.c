// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rospi_pre:srv/ChassisManager.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rospi_pre/srv/detail/chassis_manager__rosidl_typesupport_introspection_c.h"
#include "rospi_pre/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rospi_pre/srv/detail/chassis_manager__functions.h"
#include "rospi_pre/srv/detail/chassis_manager__struct.h"


// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rospi_pre__srv__ChassisManager_Request__init(message_memory);
}

void rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_fini_function(void * message_memory)
{
  rospi_pre__srv__ChassisManager_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_message_member_array[2] = {
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rospi_pre__srv__ChassisManager_Request, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "move_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rospi_pre__srv__ChassisManager_Request, move_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_message_members = {
  "rospi_pre__srv",  // message namespace
  "ChassisManager_Request",  // message name
  2,  // number of fields
  sizeof(rospi_pre__srv__ChassisManager_Request),
  rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_message_member_array,  // message members
  rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_message_type_support_handle = {
  0,
  &rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rospi_pre
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rospi_pre, srv, ChassisManager_Request)() {
  if (!rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_message_type_support_handle.typesupport_identifier) {
    rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rospi_pre__srv__ChassisManager_Request__rosidl_typesupport_introspection_c__ChassisManager_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rospi_pre/srv/detail/chassis_manager__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rospi_pre/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rospi_pre/srv/detail/chassis_manager__functions.h"
// already included above
// #include "rospi_pre/srv/detail/chassis_manager__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rospi_pre__srv__ChassisManager_Response__init(message_memory);
}

void rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_fini_function(void * message_memory)
{
  rospi_pre__srv__ChassisManager_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_message_member_array[1] = {
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rospi_pre__srv__ChassisManager_Response, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_message_members = {
  "rospi_pre__srv",  // message namespace
  "ChassisManager_Response",  // message name
  1,  // number of fields
  sizeof(rospi_pre__srv__ChassisManager_Response),
  rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_message_member_array,  // message members
  rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_message_type_support_handle = {
  0,
  &rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rospi_pre
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rospi_pre, srv, ChassisManager_Response)() {
  if (!rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_message_type_support_handle.typesupport_identifier) {
    rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rospi_pre__srv__ChassisManager_Response__rosidl_typesupport_introspection_c__ChassisManager_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rospi_pre/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rospi_pre/srv/detail/chassis_manager__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_service_members = {
  "rospi_pre__srv",  // service namespace
  "ChassisManager",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_Request_message_type_support_handle,
  NULL  // response message
  // rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_Response_message_type_support_handle
};

static rosidl_service_type_support_t rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_service_type_support_handle = {
  0,
  &rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rospi_pre, srv, ChassisManager_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rospi_pre, srv, ChassisManager_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rospi_pre
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rospi_pre, srv, ChassisManager)() {
  if (!rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_service_type_support_handle.typesupport_identifier) {
    rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rospi_pre, srv, ChassisManager_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rospi_pre, srv, ChassisManager_Response)()->data;
  }

  return &rospi_pre__srv__detail__chassis_manager__rosidl_typesupport_introspection_c__ChassisManager_service_type_support_handle;
}
