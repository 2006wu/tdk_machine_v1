#ifndef ROSPI_PRE__VISIBILITY_CONTROL_H_
#define ROSPI_PRE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSPI_PRE_EXPORT __attribute__ ((dllexport))
    #define ROSPI_PRE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSPI_PRE_EXPORT __declspec(dllexport)
    #define ROSPI_PRE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSPI_PRE_BUILDING_LIBRARY
    #define ROSPI_PRE_PUBLIC ROSPI_PRE_EXPORT
  #else
    #define ROSPI_PRE_PUBLIC ROSPI_PRE_IMPORT
  #endif
  #define ROSPI_PRE_PUBLIC_TYPE ROSPI_PRE_PUBLIC
  #define ROSPI_PRE_LOCAL
#else
  #define ROSPI_PRE_EXPORT __attribute__ ((visibility("default")))
  #define ROSPI_PRE_IMPORT
  #if __GNUC__ >= 4
    #define ROSPI_PRE_PUBLIC __attribute__ ((visibility("default")))
    #define ROSPI_PRE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSPI_PRE_PUBLIC
    #define ROSPI_PRE_LOCAL
  #endif
  #define ROSPI_PRE_PUBLIC_TYPE
#endif

#endif  // ROSPI_PRE__VISIBILITY_CONTROL_H_
