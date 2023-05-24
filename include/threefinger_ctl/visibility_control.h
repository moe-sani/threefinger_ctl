#ifndef THREEFINGER_CTL__VISIBILITY_CONTROL_H_
#define THREEFINGER_CTL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define THREEFINGER_CTL_EXPORT __attribute__ ((dllexport))
    #define THREEFINGER_CTL_IMPORT __attribute__ ((dllimport))
  #else
    #define THREEFINGER_CTL_EXPORT __declspec(dllexport)
    #define THREEFINGER_CTL_IMPORT __declspec(dllimport)
  #endif
  #ifdef THREEFINGER_CTL_BUILDING_LIBRARY
    #define THREEFINGER_CTL_PUBLIC THREEFINGER_CTL_EXPORT
  #else
    #define THREEFINGER_CTL_PUBLIC THREEFINGER_CTL_IMPORT
  #endif
  #define THREEFINGER_CTL_PUBLIC_TYPE THREEFINGER_CTL_PUBLIC
  #define THREEFINGER_CTL_LOCAL
#else
  #define THREEFINGER_CTL_EXPORT __attribute__ ((visibility("default")))
  #define THREEFINGER_CTL_IMPORT
  #if __GNUC__ >= 4
    #define THREEFINGER_CTL_PUBLIC __attribute__ ((visibility("default")))
    #define THREEFINGER_CTL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define THREEFINGER_CTL_PUBLIC
    #define THREEFINGER_CTL_LOCAL
  #endif
  #define THREEFINGER_CTL_PUBLIC_TYPE
#endif

#endif  // THREEFINGER_CTL__VISIBILITY_CONTROL_H_
