// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from communication_msgs:msg/Talker.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TALKER__STRUCT_H_
#define COMMUNICATION_MSGS__MSG__DETAIL__TALKER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'talker_name'
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Talker in the package communication_msgs.
typedef struct communication_msgs__msg__Talker
{
  rosidl_runtime_c__String talker_name;
  rosidl_runtime_c__String message;
} communication_msgs__msg__Talker;

// Struct for a sequence of communication_msgs__msg__Talker.
typedef struct communication_msgs__msg__Talker__Sequence
{
  communication_msgs__msg__Talker * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} communication_msgs__msg__Talker__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TALKER__STRUCT_H_
