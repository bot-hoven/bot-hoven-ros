#ifndef HARDWARE__CL42T__VISIBILITY_CONTROL_H
#define HARDWARE__CL42T__VISIBILITY_CONTROL_H

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CL42T_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define CL42T_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define CL42T_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define CL42T_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef CL42T_HARDWARE_INTERFACE_BUILDING_DLL
#define CL42T_HARDWARE_INTERFACE_PUBLIC CL42T_HARDWARE_INTERFACE_EXPORT
#else
#define CL42T_HARDWARE_INTERFACE_PUBLIC CL42T_HARDWARE_INTERFACE_IMPORT
#endif
#define CL42T_HARDWARE_INTERFACE_PUBLIC_TYPE CL42T_HARDWARE_INTERFACE_PUBLIC
#define CL42T_HARDWARE_INTERFACE_LOCAL
#else
#define CL42T_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define CL42T_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define CL42T_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define CL42T_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define CL42T_HARDWARE_INTERFACE_PUBLIC
#define CL42T_HARDWARE_INTERFACE_LOCAL
#endif
#define CL42T_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // HARDWARE__CL42T__VISIBILITY_CONTROL_H
