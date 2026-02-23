//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: KDAS_mat_pp_waypoints_types.h
//
// Code generated for Simulink model 'KDAS_mat_pp_waypoints'.
//
// Model version                  : 3.9
// Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
// C/C++ source code generated on : Wed Jan 28 20:19:32 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Emulation hardware selection:
//    Differs from embedded hardware (Custom Processor->Custom Processor)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef KDAS_mat_pp_waypoints_types_h_
#define KDAS_mat_pp_waypoints_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
struct SL_Bus_geometry_msgs_Point
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Int32_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Int32_

// MsgType=std_msgs/Int32
struct SL_Bus_std_msgs_Int32
{
  int32_T data;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_MultiArrayDimension_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_MultiArrayDimension_

// MsgType=std_msgs/MultiArrayDimension
struct SL_Bus_std_msgs_MultiArrayDimension
{
  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=label_SL_Info:TruncateAction=warn 
  uint8_T label[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=label
  SL_Bus_ROSVariableLengthArrayInfo label_SL_Info;
  uint32_T size;
  uint32_T stride;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_MultiArrayLayout_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_MultiArrayLayout_

// MsgType=std_msgs/MultiArrayLayout
struct SL_Bus_std_msgs_MultiArrayLayout
{
  // MsgType=std_msgs/MultiArrayDimension:IsVarLen=1:VarLenCategory=data:VarLenElem=dim_SL_Info:TruncateAction=warn 
  SL_Bus_std_msgs_MultiArrayDimension dim[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=dim
  SL_Bus_ROSVariableLengthArrayInfo dim_SL_Info;
  uint32_T data_offset;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64MultiArray_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64MultiArray_

// MsgType=std_msgs/Float64MultiArray
struct SL_Bus_std_msgs_Float64MultiArray
{
  // MsgType=std_msgs/MultiArrayLayout
  SL_Bus_std_msgs_MultiArrayLayout layout;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=data_SL_Info:TruncateAction=warn
  real_T data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=data
  SL_Bus_ROSVariableLengthArrayInfo data_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64_

// MsgType=std_msgs/Float64
struct SL_Bus_std_msgs_Float64
{
  real_T data;
};

#endif

#ifndef struct_sJ4ih70VmKcvCeguWN0mNVF
#define struct_sJ4ih70VmKcvCeguWN0mNVF

struct sJ4ih70VmKcvCeguWN0mNVF
{
  real_T sec;
  real_T nsec;
};

#endif                                 // struct_sJ4ih70VmKcvCeguWN0mNVF

#ifndef struct_ros_slros2_internal_block_Pub_T
#define struct_ros_slros2_internal_block_Pub_T

struct ros_slros2_internal_block_Pub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                                // struct_ros_slros2_internal_block_Pub_T

#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                                // struct_ros_slros2_internal_block_Sub_T

#ifndef struct_cell_wrap_KDAS_mat_pp_waypoin_T
#define struct_cell_wrap_KDAS_mat_pp_waypoin_T

struct cell_wrap_KDAS_mat_pp_waypoin_T
{
  uint32_T f1[8];
};

#endif                                // struct_cell_wrap_KDAS_mat_pp_waypoin_T

#ifndef struct_nav_slalgs_internal_PurePursu_T
#define struct_nav_slalgs_internal_PurePursu_T

struct nav_slalgs_internal_PurePursu_T
{
  int32_T isInitialized;
  cell_wrap_KDAS_mat_pp_waypoin_T inputVarSize[4];
  real_T MaxAngularVelocity;
  real_T LookaheadDistance;
  real_T DesiredLinearVelocity;
  real_T ProjectionPoint[2];
  real_T ProjectionLineIndex;
  real_T LookaheadPoint[2];
  real_T LastPose[3];
  real_T WaypointsInternal[128];
};

#endif                                // struct_nav_slalgs_internal_PurePursu_T

// Forward declaration for rtModel
typedef struct tag_RTM_KDAS_mat_pp_waypoints_T RT_MODEL_KDAS_mat_pp_waypoint_T;

#endif                                 // KDAS_mat_pp_waypoints_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
