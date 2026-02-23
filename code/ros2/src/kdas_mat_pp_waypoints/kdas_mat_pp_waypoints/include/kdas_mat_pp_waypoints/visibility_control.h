#ifndef KDAS_MAT_PP_WAYPOINTS__VISIBILITY_CONTROL_H_
#define KDAS_MAT_PP_WAYPOINTS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KDAS_MAT_PP_WAYPOINTS_EXPORT __attribute__ ((dllexport))
    #define KDAS_MAT_PP_WAYPOINTS_IMPORT __attribute__ ((dllimport))
  #else
    #define KDAS_MAT_PP_WAYPOINTS_EXPORT __declspec(dllexport)
    #define KDAS_MAT_PP_WAYPOINTS_IMPORT __declspec(dllimport)
  #endif
  #ifdef KDAS_MAT_PP_WAYPOINTS_BUILDING_LIBRARY
    #define KDAS_MAT_PP_WAYPOINTS_PUBLIC KDAS_MAT_PP_WAYPOINTS_EXPORT
  #else
    #define KDAS_MAT_PP_WAYPOINTS_PUBLIC KDAS_MAT_PP_WAYPOINTS_IMPORT
  #endif
  #define KDAS_MAT_PP_WAYPOINTS_PUBLIC_TYPE KDAS_MAT_PP_WAYPOINTS_PUBLIC
  #define KDAS_MAT_PP_WAYPOINTS_LOCAL
#else
  #define KDAS_MAT_PP_WAYPOINTS_EXPORT __attribute__ ((visibility("default")))
  #define KDAS_MAT_PP_WAYPOINTS_IMPORT
  #if __GNUC__ >= 4
    #define KDAS_MAT_PP_WAYPOINTS_PUBLIC __attribute__ ((visibility("default")))
    #define KDAS_MAT_PP_WAYPOINTS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KDAS_MAT_PP_WAYPOINTS_PUBLIC
    #define KDAS_MAT_PP_WAYPOINTS_LOCAL
  #endif
  #define KDAS_MAT_PP_WAYPOINTS_PUBLIC_TYPE
#endif
#endif  // KDAS_MAT_PP_WAYPOINTS__VISIBILITY_CONTROL_H_
// Generated 28-Jan-2026 20:19:39
 