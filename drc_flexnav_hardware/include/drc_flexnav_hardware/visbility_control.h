#ifndef DRC_FLEXNAV_HARDWARE__VISIBILITY_CONTROL_H_
#define DRC_FLEXNAV_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DRC_FLEXNAV_HARDWARE_EXPORT __attribute__((dllexport))
#define DRC_FLEXNAV_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define DRC_FLEXNAV_HARDWARE_EXPORT __declspec(dllexport)
#define DRC_FLEXNAV_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef DRC_FLEXNAV_HARDWARE_BUILDING_LIBRARY
#define DRC_FLEXNAV_HARDWARE_PUBLIC DRC_FLEXNAV_HARDWARE_EXPORT
#else
#define DRC_FLEXNAV_HARDWARE_PUBLIC DRC_FLEXNAV_HARDWARE_IMPORT
#endif
#define DRC_FLEXNAV_HARDWARE_PUBLIC_TYPE DRC_FLEXNAV_HARDWARE_PUBLIC
#define DRC_FLEXNAV_HARDWARE_LOCAL
#else
#define DRC_FLEXNAV_HARDWARE_EXPORT __attribute__((visibility("default")))
#define DRC_FLEXNAV_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define DRC_FLEXNAV_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define DRC_FLEXNAV_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define DRC_FLEXNAV_HARDWAR_CONTROLLER_PUBLIC
#define DRC_FLEXNAV_HARDWARE_CONTROLLER_LOCAL
#endif
#define DRC_FLEXNAV_HARDWARE_CONTROLLER_PUBLIC_TYPE
#endif

#endif // DRC_FLEXNAV_HARDWARE__VISIBILITY_CONTROL_H_
