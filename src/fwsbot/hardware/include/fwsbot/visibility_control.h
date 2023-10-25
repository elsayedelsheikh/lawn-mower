#ifndef FWSBOT__VISIBILITY_CONTROL_H_
#define FWSBOT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCA9685_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define PCA9685_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define PCA9685_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define PCA9685_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef PCA9685_HARDWARE_INTERFACE_BUILDING_DLL
#define PCA9685_HARDWARE_INTERFACE_PUBLIC PCA9685_HARDWARE_INTERFACE_EXPORT
#else
#define PCA9685_HARDWARE_INTERFACE_PUBLIC PCA9685_HARDWARE_INTERFACE_IMPORT
#endif
#define PCA9685_HARDWARE_INTERFACE_PUBLIC_TYPE PCA9685_HARDWARE_INTERFACE_PUBLIC
#define PCA9685_HARDWARE_INTERFACE_LOCAL
#else
#define PCA9685_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define PCA9685_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define PCA9685_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define PCA9685_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define PCA9685_HARDWARE_INTERFACE_PUBLIC
#define PCA9685_HARDWARE_INTERFACE_LOCAL
#endif
#define PCA9685_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // FWSBOT__VISIBILITY_CONTROL_H_
