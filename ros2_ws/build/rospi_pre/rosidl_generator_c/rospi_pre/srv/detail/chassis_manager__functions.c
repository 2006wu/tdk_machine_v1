// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rospi_pre:srv/ChassisManager.idl
// generated code does not contain a copyright notice
#include "rospi_pre/srv/detail/chassis_manager__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

bool
rospi_pre__srv__ChassisManager_Request__init(rospi_pre__srv__ChassisManager_Request * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    rospi_pre__srv__ChassisManager_Request__fini(msg);
    return false;
  }
  // move_type
  return true;
}

void
rospi_pre__srv__ChassisManager_Request__fini(rospi_pre__srv__ChassisManager_Request * msg)
{
  if (!msg) {
    return;
  }
  // command
  rosidl_runtime_c__String__fini(&msg->command);
  // move_type
}

bool
rospi_pre__srv__ChassisManager_Request__are_equal(const rospi_pre__srv__ChassisManager_Request * lhs, const rospi_pre__srv__ChassisManager_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  // move_type
  if (lhs->move_type != rhs->move_type) {
    return false;
  }
  return true;
}

bool
rospi_pre__srv__ChassisManager_Request__copy(
  const rospi_pre__srv__ChassisManager_Request * input,
  rospi_pre__srv__ChassisManager_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  // move_type
  output->move_type = input->move_type;
  return true;
}

rospi_pre__srv__ChassisManager_Request *
rospi_pre__srv__ChassisManager_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rospi_pre__srv__ChassisManager_Request * msg = (rospi_pre__srv__ChassisManager_Request *)allocator.allocate(sizeof(rospi_pre__srv__ChassisManager_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rospi_pre__srv__ChassisManager_Request));
  bool success = rospi_pre__srv__ChassisManager_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rospi_pre__srv__ChassisManager_Request__destroy(rospi_pre__srv__ChassisManager_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rospi_pre__srv__ChassisManager_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rospi_pre__srv__ChassisManager_Request__Sequence__init(rospi_pre__srv__ChassisManager_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rospi_pre__srv__ChassisManager_Request * data = NULL;

  if (size) {
    data = (rospi_pre__srv__ChassisManager_Request *)allocator.zero_allocate(size, sizeof(rospi_pre__srv__ChassisManager_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rospi_pre__srv__ChassisManager_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rospi_pre__srv__ChassisManager_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rospi_pre__srv__ChassisManager_Request__Sequence__fini(rospi_pre__srv__ChassisManager_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rospi_pre__srv__ChassisManager_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rospi_pre__srv__ChassisManager_Request__Sequence *
rospi_pre__srv__ChassisManager_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rospi_pre__srv__ChassisManager_Request__Sequence * array = (rospi_pre__srv__ChassisManager_Request__Sequence *)allocator.allocate(sizeof(rospi_pre__srv__ChassisManager_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rospi_pre__srv__ChassisManager_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rospi_pre__srv__ChassisManager_Request__Sequence__destroy(rospi_pre__srv__ChassisManager_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rospi_pre__srv__ChassisManager_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rospi_pre__srv__ChassisManager_Request__Sequence__are_equal(const rospi_pre__srv__ChassisManager_Request__Sequence * lhs, const rospi_pre__srv__ChassisManager_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rospi_pre__srv__ChassisManager_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rospi_pre__srv__ChassisManager_Request__Sequence__copy(
  const rospi_pre__srv__ChassisManager_Request__Sequence * input,
  rospi_pre__srv__ChassisManager_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rospi_pre__srv__ChassisManager_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rospi_pre__srv__ChassisManager_Request * data =
      (rospi_pre__srv__ChassisManager_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rospi_pre__srv__ChassisManager_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rospi_pre__srv__ChassisManager_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rospi_pre__srv__ChassisManager_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
rospi_pre__srv__ChassisManager_Response__init(rospi_pre__srv__ChassisManager_Response * msg)
{
  if (!msg) {
    return false;
  }
  // valid
  return true;
}

void
rospi_pre__srv__ChassisManager_Response__fini(rospi_pre__srv__ChassisManager_Response * msg)
{
  if (!msg) {
    return;
  }
  // valid
}

bool
rospi_pre__srv__ChassisManager_Response__are_equal(const rospi_pre__srv__ChassisManager_Response * lhs, const rospi_pre__srv__ChassisManager_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  return true;
}

bool
rospi_pre__srv__ChassisManager_Response__copy(
  const rospi_pre__srv__ChassisManager_Response * input,
  rospi_pre__srv__ChassisManager_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // valid
  output->valid = input->valid;
  return true;
}

rospi_pre__srv__ChassisManager_Response *
rospi_pre__srv__ChassisManager_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rospi_pre__srv__ChassisManager_Response * msg = (rospi_pre__srv__ChassisManager_Response *)allocator.allocate(sizeof(rospi_pre__srv__ChassisManager_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rospi_pre__srv__ChassisManager_Response));
  bool success = rospi_pre__srv__ChassisManager_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rospi_pre__srv__ChassisManager_Response__destroy(rospi_pre__srv__ChassisManager_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rospi_pre__srv__ChassisManager_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rospi_pre__srv__ChassisManager_Response__Sequence__init(rospi_pre__srv__ChassisManager_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rospi_pre__srv__ChassisManager_Response * data = NULL;

  if (size) {
    data = (rospi_pre__srv__ChassisManager_Response *)allocator.zero_allocate(size, sizeof(rospi_pre__srv__ChassisManager_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rospi_pre__srv__ChassisManager_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rospi_pre__srv__ChassisManager_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rospi_pre__srv__ChassisManager_Response__Sequence__fini(rospi_pre__srv__ChassisManager_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rospi_pre__srv__ChassisManager_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rospi_pre__srv__ChassisManager_Response__Sequence *
rospi_pre__srv__ChassisManager_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rospi_pre__srv__ChassisManager_Response__Sequence * array = (rospi_pre__srv__ChassisManager_Response__Sequence *)allocator.allocate(sizeof(rospi_pre__srv__ChassisManager_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rospi_pre__srv__ChassisManager_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rospi_pre__srv__ChassisManager_Response__Sequence__destroy(rospi_pre__srv__ChassisManager_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rospi_pre__srv__ChassisManager_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rospi_pre__srv__ChassisManager_Response__Sequence__are_equal(const rospi_pre__srv__ChassisManager_Response__Sequence * lhs, const rospi_pre__srv__ChassisManager_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rospi_pre__srv__ChassisManager_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rospi_pre__srv__ChassisManager_Response__Sequence__copy(
  const rospi_pre__srv__ChassisManager_Response__Sequence * input,
  rospi_pre__srv__ChassisManager_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rospi_pre__srv__ChassisManager_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rospi_pre__srv__ChassisManager_Response * data =
      (rospi_pre__srv__ChassisManager_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rospi_pre__srv__ChassisManager_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rospi_pre__srv__ChassisManager_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rospi_pre__srv__ChassisManager_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
