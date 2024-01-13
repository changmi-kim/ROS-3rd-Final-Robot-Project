// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from communication_msgs:msg/Talker.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TALKER__STRUCT_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TALKER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__communication_msgs__msg__Talker __attribute__((deprecated))
#else
# define DEPRECATED__communication_msgs__msg__Talker __declspec(deprecated)
#endif

namespace communication_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Talker_
{
  using Type = Talker_<ContainerAllocator>;

  explicit Talker_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->talker_name = "";
      this->message = "";
    }
  }

  explicit Talker_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : talker_name(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->talker_name = "";
      this->message = "";
    }
  }

  // field types and members
  using _talker_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _talker_name_type talker_name;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__talker_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->talker_name = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    communication_msgs::msg::Talker_<ContainerAllocator> *;
  using ConstRawPtr =
    const communication_msgs::msg::Talker_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<communication_msgs::msg::Talker_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<communication_msgs::msg::Talker_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::Talker_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::Talker_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::Talker_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::Talker_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<communication_msgs::msg::Talker_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<communication_msgs::msg::Talker_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__communication_msgs__msg__Talker
    std::shared_ptr<communication_msgs::msg::Talker_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__communication_msgs__msg__Talker
    std::shared_ptr<communication_msgs::msg::Talker_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Talker_ & other) const
  {
    if (this->talker_name != other.talker_name) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Talker_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Talker_

// alias to use template instance with default allocator
using Talker =
  communication_msgs::msg::Talker_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TALKER__STRUCT_HPP_
