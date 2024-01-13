// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from communication_msgs:msg/Talker.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TALKER__BUILDER_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TALKER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "communication_msgs/msg/detail/talker__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace communication_msgs
{

namespace msg
{

namespace builder
{

class Init_Talker_message
{
public:
  explicit Init_Talker_message(::communication_msgs::msg::Talker & msg)
  : msg_(msg)
  {}
  ::communication_msgs::msg::Talker message(::communication_msgs::msg::Talker::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::communication_msgs::msg::Talker msg_;
};

class Init_Talker_talker_name
{
public:
  Init_Talker_talker_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Talker_message talker_name(::communication_msgs::msg::Talker::_talker_name_type arg)
  {
    msg_.talker_name = std::move(arg);
    return Init_Talker_message(msg_);
  }

private:
  ::communication_msgs::msg::Talker msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::communication_msgs::msg::Talker>()
{
  return communication_msgs::msg::builder::Init_Talker_talker_name();
}

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TALKER__BUILDER_HPP_
