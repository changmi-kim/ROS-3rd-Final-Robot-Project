// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from communication_msgs:msg/Talker.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TALKER__FUNCTIONS_H_
#define COMMUNICATION_MSGS__MSG__DETAIL__TALKER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "communication_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "communication_msgs/msg/detail/talker__struct.h"

/// Initialize msg/Talker message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * communication_msgs__msg__Talker
 * )) before or use
 * communication_msgs__msg__Talker__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__Talker__init(communication_msgs__msg__Talker * msg);

/// Finalize msg/Talker message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
void
communication_msgs__msg__Talker__fini(communication_msgs__msg__Talker * msg);

/// Create msg/Talker message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * communication_msgs__msg__Talker__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
communication_msgs__msg__Talker *
communication_msgs__msg__Talker__create();

/// Destroy msg/Talker message.
/**
 * It calls
 * communication_msgs__msg__Talker__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
void
communication_msgs__msg__Talker__destroy(communication_msgs__msg__Talker * msg);

/// Check for msg/Talker message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__Talker__are_equal(const communication_msgs__msg__Talker * lhs, const communication_msgs__msg__Talker * rhs);

/// Copy a msg/Talker message.
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
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__Talker__copy(
  const communication_msgs__msg__Talker * input,
  communication_msgs__msg__Talker * output);

/// Initialize array of msg/Talker messages.
/**
 * It allocates the memory for the number of elements and calls
 * communication_msgs__msg__Talker__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__Talker__Sequence__init(communication_msgs__msg__Talker__Sequence * array, size_t size);

/// Finalize array of msg/Talker messages.
/**
 * It calls
 * communication_msgs__msg__Talker__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
void
communication_msgs__msg__Talker__Sequence__fini(communication_msgs__msg__Talker__Sequence * array);

/// Create array of msg/Talker messages.
/**
 * It allocates the memory for the array and calls
 * communication_msgs__msg__Talker__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
communication_msgs__msg__Talker__Sequence *
communication_msgs__msg__Talker__Sequence__create(size_t size);

/// Destroy array of msg/Talker messages.
/**
 * It calls
 * communication_msgs__msg__Talker__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
void
communication_msgs__msg__Talker__Sequence__destroy(communication_msgs__msg__Talker__Sequence * array);

/// Check for msg/Talker message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__Talker__Sequence__are_equal(const communication_msgs__msg__Talker__Sequence * lhs, const communication_msgs__msg__Talker__Sequence * rhs);

/// Copy an array of msg/Talker messages.
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
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__Talker__Sequence__copy(
  const communication_msgs__msg__Talker__Sequence * input,
  communication_msgs__msg__Talker__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TALKER__FUNCTIONS_H_
