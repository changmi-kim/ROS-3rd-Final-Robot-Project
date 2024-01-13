// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from communication_msgs:msg/Talker.idl
// generated code does not contain a copyright notice
#include "communication_msgs/msg/detail/talker__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `talker_name`
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
communication_msgs__msg__Talker__init(communication_msgs__msg__Talker * msg)
{
  if (!msg) {
    return false;
  }
  // talker_name
  if (!rosidl_runtime_c__String__init(&msg->talker_name)) {
    communication_msgs__msg__Talker__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    communication_msgs__msg__Talker__fini(msg);
    return false;
  }
  return true;
}

void
communication_msgs__msg__Talker__fini(communication_msgs__msg__Talker * msg)
{
  if (!msg) {
    return;
  }
  // talker_name
  rosidl_runtime_c__String__fini(&msg->talker_name);
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
communication_msgs__msg__Talker__are_equal(const communication_msgs__msg__Talker * lhs, const communication_msgs__msg__Talker * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // talker_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->talker_name), &(rhs->talker_name)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
communication_msgs__msg__Talker__copy(
  const communication_msgs__msg__Talker * input,
  communication_msgs__msg__Talker * output)
{
  if (!input || !output) {
    return false;
  }
  // talker_name
  if (!rosidl_runtime_c__String__copy(
      &(input->talker_name), &(output->talker_name)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

communication_msgs__msg__Talker *
communication_msgs__msg__Talker__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  communication_msgs__msg__Talker * msg = (communication_msgs__msg__Talker *)allocator.allocate(sizeof(communication_msgs__msg__Talker), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(communication_msgs__msg__Talker));
  bool success = communication_msgs__msg__Talker__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
communication_msgs__msg__Talker__destroy(communication_msgs__msg__Talker * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    communication_msgs__msg__Talker__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
communication_msgs__msg__Talker__Sequence__init(communication_msgs__msg__Talker__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  communication_msgs__msg__Talker * data = NULL;

  if (size) {
    data = (communication_msgs__msg__Talker *)allocator.zero_allocate(size, sizeof(communication_msgs__msg__Talker), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = communication_msgs__msg__Talker__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        communication_msgs__msg__Talker__fini(&data[i - 1]);
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
communication_msgs__msg__Talker__Sequence__fini(communication_msgs__msg__Talker__Sequence * array)
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
      communication_msgs__msg__Talker__fini(&array->data[i]);
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

communication_msgs__msg__Talker__Sequence *
communication_msgs__msg__Talker__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  communication_msgs__msg__Talker__Sequence * array = (communication_msgs__msg__Talker__Sequence *)allocator.allocate(sizeof(communication_msgs__msg__Talker__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = communication_msgs__msg__Talker__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
communication_msgs__msg__Talker__Sequence__destroy(communication_msgs__msg__Talker__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    communication_msgs__msg__Talker__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
communication_msgs__msg__Talker__Sequence__are_equal(const communication_msgs__msg__Talker__Sequence * lhs, const communication_msgs__msg__Talker__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!communication_msgs__msg__Talker__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
communication_msgs__msg__Talker__Sequence__copy(
  const communication_msgs__msg__Talker__Sequence * input,
  communication_msgs__msg__Talker__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(communication_msgs__msg__Talker);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    communication_msgs__msg__Talker * data =
      (communication_msgs__msg__Talker *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!communication_msgs__msg__Talker__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          communication_msgs__msg__Talker__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!communication_msgs__msg__Talker__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
