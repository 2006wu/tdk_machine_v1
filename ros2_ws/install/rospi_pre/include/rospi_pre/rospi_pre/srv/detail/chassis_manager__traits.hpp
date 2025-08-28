// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rospi_pre:srv/ChassisManager.idl
// generated code does not contain a copyright notice

#ifndef ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__TRAITS_HPP_
#define ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rospi_pre/srv/detail/chassis_manager__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rospi_pre
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChassisManager_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: move_type
  {
    out << "move_type: ";
    rosidl_generator_traits::value_to_yaml(msg.move_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChassisManager_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: move_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_type: ";
    rosidl_generator_traits::value_to_yaml(msg.move_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChassisManager_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rospi_pre

namespace rosidl_generator_traits
{

[[deprecated("use rospi_pre::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rospi_pre::srv::ChassisManager_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rospi_pre::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rospi_pre::srv::to_yaml() instead")]]
inline std::string to_yaml(const rospi_pre::srv::ChassisManager_Request & msg)
{
  return rospi_pre::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rospi_pre::srv::ChassisManager_Request>()
{
  return "rospi_pre::srv::ChassisManager_Request";
}

template<>
inline const char * name<rospi_pre::srv::ChassisManager_Request>()
{
  return "rospi_pre/srv/ChassisManager_Request";
}

template<>
struct has_fixed_size<rospi_pre::srv::ChassisManager_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rospi_pre::srv::ChassisManager_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rospi_pre::srv::ChassisManager_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rospi_pre
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChassisManager_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChassisManager_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChassisManager_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rospi_pre

namespace rosidl_generator_traits
{

[[deprecated("use rospi_pre::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rospi_pre::srv::ChassisManager_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rospi_pre::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rospi_pre::srv::to_yaml() instead")]]
inline std::string to_yaml(const rospi_pre::srv::ChassisManager_Response & msg)
{
  return rospi_pre::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rospi_pre::srv::ChassisManager_Response>()
{
  return "rospi_pre::srv::ChassisManager_Response";
}

template<>
inline const char * name<rospi_pre::srv::ChassisManager_Response>()
{
  return "rospi_pre/srv/ChassisManager_Response";
}

template<>
struct has_fixed_size<rospi_pre::srv::ChassisManager_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rospi_pre::srv::ChassisManager_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rospi_pre::srv::ChassisManager_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rospi_pre::srv::ChassisManager>()
{
  return "rospi_pre::srv::ChassisManager";
}

template<>
inline const char * name<rospi_pre::srv::ChassisManager>()
{
  return "rospi_pre/srv/ChassisManager";
}

template<>
struct has_fixed_size<rospi_pre::srv::ChassisManager>
  : std::integral_constant<
    bool,
    has_fixed_size<rospi_pre::srv::ChassisManager_Request>::value &&
    has_fixed_size<rospi_pre::srv::ChassisManager_Response>::value
  >
{
};

template<>
struct has_bounded_size<rospi_pre::srv::ChassisManager>
  : std::integral_constant<
    bool,
    has_bounded_size<rospi_pre::srv::ChassisManager_Request>::value &&
    has_bounded_size<rospi_pre::srv::ChassisManager_Response>::value
  >
{
};

template<>
struct is_service<rospi_pre::srv::ChassisManager>
  : std::true_type
{
};

template<>
struct is_service_request<rospi_pre::srv::ChassisManager_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rospi_pre::srv::ChassisManager_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSPI_PRE__SRV__DETAIL__CHASSIS_MANAGER__TRAITS_HPP_
