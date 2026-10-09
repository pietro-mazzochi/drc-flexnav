#ifndef ODOMETRY_ARC__VISIBILITY_CONTROL_H_
#define ODOMETRY_ARC__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ODOMETRY_ARC_EXPORT __attribute__((dllexport))
#define ODOMETRY_ARC_IMPORT __attribute__((dllimport))
#else
#define ODOMETRY_ARC_EXPORT __declspec(dllexport)
#define ODOMETRY_ARC_IMPORT __declspec(dllimport)
#endif
#ifdef ODOMETRY_ARC_BUILDING_LIBRARY
#define ODOMETRY_ARC_PUBLIC ODOMETRY_ARC_EXPORT
#else
#define ODOMETRY_ARC_PUBLIC ODOMETRY_ARC_IMPORT
#endif
#define ODOMETRY_ARC_PUBLIC_TYPE ODOMETRY_ARC_PUBLIC
#define ODOMETRY_ARC_LOCAL
#else
#define ODOMETRY_ARC_EXPORT __attribute__((visibility("default")))
#define ODOMETRY_ARC_IMPORT
#if __GNUC__ >= 4
#define ODOMETRY_ARC_PUBLIC __attribute__((visibility("default")))
#define ODOMETRY_ARC_LOCAL __attribute__((visibility("hidden")))
#else
#define ODOMETRY_ARC_PUBLIC
#define ODOMETRY_ARC_LOCAL
#endif
#define ODOMETRY_ARC_PUBLIC_TYPE
#endif

#endif // ODOMETRY_ARC__VISIBILITY_CONTROL_H_