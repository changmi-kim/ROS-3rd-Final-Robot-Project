// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from communication_msgs:msg/Talker.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TALKER__TRAITS_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TALKER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "communication_msgs/msg/detail/talker__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace communication_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Talker & msg,
  std::ostream & out)
{
  out << "{";
  // member: talker_name
  {
    out << "talker_name: ";
    rosidl_generator_traits::value_to_yaml(msg.talker_name, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Talker & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: talker_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "talker_name: ";
    rosidl_generator_traits::value_to_yaml(msg.talker_name, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Talker & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace communication_msgs

namespace rosidl_generator_traits
{

[[deprecated("use communication_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const communication_msgs::msg::Talker & msg,
  std::ostream & out, size_t indentation = 0)
{
  communication_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use communication_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const communication_msgs::msg::Talker & msg)
{
  return communication_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<communication_msgs::msg::Talker>()
{
  return "communication_msgs::msg::Talker";
}

template<>
inline const char * name<communication_msgs::msg::Talker>()
{
  return "communication_msgs/msg/Talker";
}

template<>
struct has_fixed_size<communication_msgs::msg::Talker>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<communication_msgs::msg::Talker>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<communication_msgs::msg::Talker>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TALKER__TRAITS_HPP_
