// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from maki_interfaces:srv/HapticFeedback.idl
// generated code does not contain a copyright notice
#include "maki_interfaces/srv/detail/haptic_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
maki_interfaces__srv__HapticFeedback_Request__init(maki_interfaces__srv__HapticFeedback_Request * msg)
{
  if (!msg) {
    return false;
  }
  // intensity
  // duration
  return true;
}

void
maki_interfaces__srv__HapticFeedback_Request__fini(maki_interfaces__srv__HapticFeedback_Request * msg)
{
  if (!msg) {
    return;
  }
  // intensity
  // duration
}

bool
maki_interfaces__srv__HapticFeedback_Request__are_equal(const maki_interfaces__srv__HapticFeedback_Request * lhs, const maki_interfaces__srv__HapticFeedback_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // intensity
  if (lhs->intensity != rhs->intensity) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  return true;
}

bool
maki_interfaces__srv__HapticFeedback_Request__copy(
  const maki_interfaces__srv__HapticFeedback_Request * input,
  maki_interfaces__srv__HapticFeedback_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // intensity
  output->intensity = input->intensity;
  // duration
  output->duration = input->duration;
  return true;
}

maki_interfaces__srv__HapticFeedback_Request *
maki_interfaces__srv__HapticFeedback_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maki_interfaces__srv__HapticFeedback_Request * msg = (maki_interfaces__srv__HapticFeedback_Request *)allocator.allocate(sizeof(maki_interfaces__srv__HapticFeedback_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maki_interfaces__srv__HapticFeedback_Request));
  bool success = maki_interfaces__srv__HapticFeedback_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maki_interfaces__srv__HapticFeedback_Request__destroy(maki_interfaces__srv__HapticFeedback_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maki_interfaces__srv__HapticFeedback_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maki_interfaces__srv__HapticFeedback_Request__Sequence__init(maki_interfaces__srv__HapticFeedback_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maki_interfaces__srv__HapticFeedback_Request * data = NULL;

  if (size) {
    data = (maki_interfaces__srv__HapticFeedback_Request *)allocator.zero_allocate(size, sizeof(maki_interfaces__srv__HapticFeedback_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maki_interfaces__srv__HapticFeedback_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maki_interfaces__srv__HapticFeedback_Request__fini(&data[i - 1]);
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
maki_interfaces__srv__HapticFeedback_Request__Sequence__fini(maki_interfaces__srv__HapticFeedback_Request__Sequence * array)
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
      maki_interfaces__srv__HapticFeedback_Request__fini(&array->data[i]);
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

maki_interfaces__srv__HapticFeedback_Request__Sequence *
maki_interfaces__srv__HapticFeedback_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maki_interfaces__srv__HapticFeedback_Request__Sequence * array = (maki_interfaces__srv__HapticFeedback_Request__Sequence *)allocator.allocate(sizeof(maki_interfaces__srv__HapticFeedback_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maki_interfaces__srv__HapticFeedback_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maki_interfaces__srv__HapticFeedback_Request__Sequence__destroy(maki_interfaces__srv__HapticFeedback_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maki_interfaces__srv__HapticFeedback_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maki_interfaces__srv__HapticFeedback_Request__Sequence__are_equal(const maki_interfaces__srv__HapticFeedback_Request__Sequence * lhs, const maki_interfaces__srv__HapticFeedback_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maki_interfaces__srv__HapticFeedback_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maki_interfaces__srv__HapticFeedback_Request__Sequence__copy(
  const maki_interfaces__srv__HapticFeedback_Request__Sequence * input,
  maki_interfaces__srv__HapticFeedback_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maki_interfaces__srv__HapticFeedback_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maki_interfaces__srv__HapticFeedback_Request * data =
      (maki_interfaces__srv__HapticFeedback_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maki_interfaces__srv__HapticFeedback_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maki_interfaces__srv__HapticFeedback_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maki_interfaces__srv__HapticFeedback_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `response`
#include "rosidl_runtime_c/string_functions.h"

bool
maki_interfaces__srv__HapticFeedback_Response__init(maki_interfaces__srv__HapticFeedback_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // response
  if (!rosidl_runtime_c__String__init(&msg->response)) {
    maki_interfaces__srv__HapticFeedback_Response__fini(msg);
    return false;
  }
  return true;
}

void
maki_interfaces__srv__HapticFeedback_Response__fini(maki_interfaces__srv__HapticFeedback_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // response
  rosidl_runtime_c__String__fini(&msg->response);
}

bool
maki_interfaces__srv__HapticFeedback_Response__are_equal(const maki_interfaces__srv__HapticFeedback_Response * lhs, const maki_interfaces__srv__HapticFeedback_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // response
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
maki_interfaces__srv__HapticFeedback_Response__copy(
  const maki_interfaces__srv__HapticFeedback_Response * input,
  maki_interfaces__srv__HapticFeedback_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // response
  if (!rosidl_runtime_c__String__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

maki_interfaces__srv__HapticFeedback_Response *
maki_interfaces__srv__HapticFeedback_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maki_interfaces__srv__HapticFeedback_Response * msg = (maki_interfaces__srv__HapticFeedback_Response *)allocator.allocate(sizeof(maki_interfaces__srv__HapticFeedback_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maki_interfaces__srv__HapticFeedback_Response));
  bool success = maki_interfaces__srv__HapticFeedback_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maki_interfaces__srv__HapticFeedback_Response__destroy(maki_interfaces__srv__HapticFeedback_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maki_interfaces__srv__HapticFeedback_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maki_interfaces__srv__HapticFeedback_Response__Sequence__init(maki_interfaces__srv__HapticFeedback_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maki_interfaces__srv__HapticFeedback_Response * data = NULL;

  if (size) {
    data = (maki_interfaces__srv__HapticFeedback_Response *)allocator.zero_allocate(size, sizeof(maki_interfaces__srv__HapticFeedback_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maki_interfaces__srv__HapticFeedback_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maki_interfaces__srv__HapticFeedback_Response__fini(&data[i - 1]);
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
maki_interfaces__srv__HapticFeedback_Response__Sequence__fini(maki_interfaces__srv__HapticFeedback_Response__Sequence * array)
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
      maki_interfaces__srv__HapticFeedback_Response__fini(&array->data[i]);
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

maki_interfaces__srv__HapticFeedback_Response__Sequence *
maki_interfaces__srv__HapticFeedback_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maki_interfaces__srv__HapticFeedback_Response__Sequence * array = (maki_interfaces__srv__HapticFeedback_Response__Sequence *)allocator.allocate(sizeof(maki_interfaces__srv__HapticFeedback_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maki_interfaces__srv__HapticFeedback_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maki_interfaces__srv__HapticFeedback_Response__Sequence__destroy(maki_interfaces__srv__HapticFeedback_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maki_interfaces__srv__HapticFeedback_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maki_interfaces__srv__HapticFeedback_Response__Sequence__are_equal(const maki_interfaces__srv__HapticFeedback_Response__Sequence * lhs, const maki_interfaces__srv__HapticFeedback_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maki_interfaces__srv__HapticFeedback_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maki_interfaces__srv__HapticFeedback_Response__Sequence__copy(
  const maki_interfaces__srv__HapticFeedback_Response__Sequence * input,
  maki_interfaces__srv__HapticFeedback_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maki_interfaces__srv__HapticFeedback_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maki_interfaces__srv__HapticFeedback_Response * data =
      (maki_interfaces__srv__HapticFeedback_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maki_interfaces__srv__HapticFeedback_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maki_interfaces__srv__HapticFeedback_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maki_interfaces__srv__HapticFeedback_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
