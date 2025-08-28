// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rospi_pre:srv/ChassisManager.idl
// generated code does not contain a copyright notice

#ifndef ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__BUILDER_HPP_
#define ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rospi_pre/srv/detail/chassis_manager__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rospi_pre
{

namespace srv
{

namespace builder
{

class Init_ChassisManager_Request_move_type
{
public:
  explicit Init_ChassisManager_Request_move_type(::rospi_pre::srv::ChassisManager_Request & msg)
  : msg_(msg)
  {}
  ::rospi_pre::srv::ChassisManager_Request move_type(::rospi_pre::srv::ChassisManager_Request::_move_type_type arg)
  {
    msg_.move_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rospi_pre::srv::ChassisManager_Request msg_;
};

class Init_ChassisManager_Request_command
{
public:
  Init_ChassisManager_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChassisManager_Request_move_type command(::rospi_pre::srv::ChassisManager_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_ChassisManager_Request_move_type(msg_);
  }

private:
  ::rospi_pre::srv::ChassisManager_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rospi_pre::srv::ChassisManager_Request>()
{
  return rospi_pre::srv::builder::Init_ChassisManager_Request_command();
}

}  // namespace rospi_pre


namespace rospi_pre
{

namespace srv
{

namespace builder
{

class Init_ChassisManager_Response_valid
{
public:
  Init_ChassisManager_Response_valid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rospi_pre::srv::ChassisManager_Response valid(::rospi_pre::srv::ChassisManager_Response::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rospi_pre::srv::ChassisManager_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rospi_pre::srv::ChassisManager_Response>()
{
  return rospi_pre::srv::builder::Init_ChassisManager_Response_valid();
}

}  // namespace rospi_pre

#endif  // ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__BUILDER_HPP_
