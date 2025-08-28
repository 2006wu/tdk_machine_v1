// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rospi_pre:srv/ChassisManager.idl
// generated code does not contain a copyright notice

#ifndef ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__FUNCTIONS_H_
#define ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rospi_pre/msg/rosidl_generator_c__visibility_control.h"

#include "rospi_pre/srv/detail/chassis_manager__struct.h"

/// Initialize srv/ChassisManager message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rospi_pre__srv__ChassisManager_Request
 * )) before or use
 * rospi_pre__srv__ChassisManager_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Request__init(rospi_pre__srv__ChassisManager_Request * msg);

/// Finalize srv/ChassisManager message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
void
rospi_pre__srv__ChassisManager_Request__fini(rospi_pre__srv__ChassisManager_Request * msg);

/// Create srv/ChassisManager message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rospi_pre__srv__ChassisManager_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
rospi_pre__srv__ChassisManager_Request *
rospi_pre__srv__ChassisManager_Request__create();

/// Destroy srv/ChassisManager message.
/**
 * It calls
 * rospi_pre__srv__ChassisManager_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
void
rospi_pre__srv__ChassisManager_Request__destroy(rospi_pre__srv__ChassisManager_Request * msg);

/// Check for srv/ChassisManager message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Request__are_equal(const rospi_pre__srv__ChassisManager_Request * lhs, const rospi_pre__srv__ChassisManager_Request * rhs);

/// Copy a srv/ChassisManager message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Request__copy(
  const rospi_pre__srv__ChassisManager_Request * input,
  rospi_pre__srv__ChassisManager_Request * output);

/// Initialize array of srv/ChassisManager messages.
/**
 * It allocates the memory for the number of elements and calls
 * rospi_pre__srv__ChassisManager_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Request__Sequence__init(rospi_pre__srv__ChassisManager_Request__Sequence * array, size_t size);

/// Finalize array of srv/ChassisManager messages.
/**
 * It calls
 * rospi_pre__srv__ChassisManager_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
void
rospi_pre__srv__ChassisManager_Request__Sequence__fini(rospi_pre__srv__ChassisManager_Request__Sequence * array);

/// Create array of srv/ChassisManager messages.
/**
 * It allocates the memory for the array and calls
 * rospi_pre__srv__ChassisManager_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
rospi_pre__srv__ChassisManager_Request__Sequence *
rospi_pre__srv__ChassisManager_Request__Sequence__create(size_t size);

/// Destroy array of srv/ChassisManager messages.
/**
 * It calls
 * rospi_pre__srv__ChassisManager_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
void
rospi_pre__srv__ChassisManager_Request__Sequence__destroy(rospi_pre__srv__ChassisManager_Request__Sequence * array);

/// Check for srv/ChassisManager message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Request__Sequence__are_equal(const rospi_pre__srv__ChassisManager_Request__Sequence * lhs, const rospi_pre__srv__ChassisManager_Request__Sequence * rhs);

/// Copy an array of srv/ChassisManager messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Request__Sequence__copy(
  const rospi_pre__srv__ChassisManager_Request__Sequence * input,
  rospi_pre__srv__ChassisManager_Request__Sequence * output);

/// Initialize srv/ChassisManager message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rospi_pre__srv__ChassisManager_Response
 * )) before or use
 * rospi_pre__srv__ChassisManager_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Response__init(rospi_pre__srv__ChassisManager_Response * msg);

/// Finalize srv/ChassisManager message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
void
rospi_pre__srv__ChassisManager_Response__fini(rospi_pre__srv__ChassisManager_Response * msg);

/// Create srv/ChassisManager message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rospi_pre__srv__ChassisManager_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
rospi_pre__srv__ChassisManager_Response *
rospi_pre__srv__ChassisManager_Response__create();

/// Destroy srv/ChassisManager message.
/**
 * It calls
 * rospi_pre__srv__ChassisManager_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
void
rospi_pre__srv__ChassisManager_Response__destroy(rospi_pre__srv__ChassisManager_Response * msg);

/// Check for srv/ChassisManager message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Response__are_equal(const rospi_pre__srv__ChassisManager_Response * lhs, const rospi_pre__srv__ChassisManager_Response * rhs);

/// Copy a srv/ChassisManager message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Response__copy(
  const rospi_pre__srv__ChassisManager_Response * input,
  rospi_pre__srv__ChassisManager_Response * output);

/// Initialize array of srv/ChassisManager messages.
/**
 * It allocates the memory for the number of elements and calls
 * rospi_pre__srv__ChassisManager_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Response__Sequence__init(rospi_pre__srv__ChassisManager_Response__Sequence * array, size_t size);

/// Finalize array of srv/ChassisManager messages.
/**
 * It calls
 * rospi_pre__srv__ChassisManager_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
void
rospi_pre__srv__ChassisManager_Response__Sequence__fini(rospi_pre__srv__ChassisManager_Response__Sequence * array);

/// Create array of srv/ChassisManager messages.
/**
 * It allocates the memory for the array and calls
 * rospi_pre__srv__ChassisManager_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
rospi_pre__srv__ChassisManager_Response__Sequence *
rospi_pre__srv__ChassisManager_Response__Sequence__create(size_t size);

/// Destroy array of srv/ChassisManager messages.
/**
 * It calls
 * rospi_pre__srv__ChassisManager_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
void
rospi_pre__srv__ChassisManager_Response__Sequence__destroy(rospi_pre__srv__ChassisManager_Response__Sequence * array);

/// Check for srv/ChassisManager message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Response__Sequence__are_equal(const rospi_pre__srv__ChassisManager_Response__Sequence * lhs, const rospi_pre__srv__ChassisManager_Response__Sequence * rhs);

/// Copy an array of srv/ChassisManager messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rospi_pre
bool
rospi_pre__srv__ChassisManager_Response__Sequence__copy(
  const rospi_pre__srv__ChassisManager_Response__Sequence * input,
  rospi_pre__srv__ChassisManager_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__FUNCTIONS_H_
