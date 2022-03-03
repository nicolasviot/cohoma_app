#ifndef NAVGRAPH__VISIBILITY_CONTROL_H_
#define NAVGRAPH__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAVGRAPH_EXPORT __attribute__ ((dllexport))
    #define NAVGRAPH_IMPORT __attribute__ ((dllimport))
  #else
    #define NAVGRAPH_EXPORT __declspec(dllexport)
    #define NAVGRAPH_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAVGRAPH_BUILDING_LIBRARY
    #define NAVGRAPH_PUBLIC NAVGRAPH_EXPORT
  #else
    #define NAVGRAPH_PUBLIC NAVGRAPH_IMPORT
  #endif
  #define NAVGRAPH_PUBLIC_TYPE NAVGRAPH_PUBLIC
  #define NAVGRAPH_LOCAL
#else
  #define NAVGRAPH_EXPORT __attribute__ ((visibility("default")))
  #define NAVGRAPH_IMPORT
  #if __GNUC__ >= 4
    #define NAVGRAPH_PUBLIC __attribute__ ((visibility("default")))
    #define NAVGRAPH_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAVGRAPH_PUBLIC
    #define NAVGRAPH_LOCAL
  #endif
  #define NAVGRAPH_PUBLIC_TYPE
#endif

#endif  // NAVGRAPH__VISIBILITY_CONTROL_H_
