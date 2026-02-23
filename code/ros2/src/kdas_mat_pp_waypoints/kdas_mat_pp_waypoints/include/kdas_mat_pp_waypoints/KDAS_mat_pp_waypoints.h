//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: KDAS_mat_pp_waypoints.h
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
#ifndef KDAS_mat_pp_waypoints_h_
#define KDAS_mat_pp_waypoints_h_
#include "rtwtypes.h"
#include "slros2_initialize.h"
#include "KDAS_mat_pp_waypoints_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <stddef.h>

// Block signals (default storage)
struct B_KDAS_mat_pp_waypoints_T {
  SL_Bus_std_msgs_Float64MultiArray In1;// '<S17>/In1'
  SL_Bus_std_msgs_Float64MultiArray rtb_SourceBlock_o2_m;
  real_T W[128];                       // '<S3>/MATLAB Function'
  boolean_T b[128];
  int8_T tmp_data[64];
  boolean_T bv[64];
  SL_Bus_geometry_msgs_Point In1_g;    // '<S20>/In1'
  SL_Bus_geometry_msgs_Point BusAssignment;// '<S2>/Bus Assignment'
  SL_Bus_geometry_msgs_Point BusAssignment_d;// '<S5>/Bus Assignment'
  real_T TmpSignalConversionAtPurePu[3];
  char_T b_zeroDelimTopic[18];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_k;
  sJ4ih70VmKcvCeguWN0mNVF deadline_c;
  sJ4ih70VmKcvCeguWN0mNVF deadline_b;
  sJ4ih70VmKcvCeguWN0mNVF deadline_p;
  sJ4ih70VmKcvCeguWN0mNVF deadline_cv;
  sJ4ih70VmKcvCeguWN0mNVF deadline_f;
  sJ4ih70VmKcvCeguWN0mNVF deadline_g;
  real_T lookaheadStartPt[2];
  real_T lookaheadStartPt_c[2];
  real_T dv[2];
  real_T refPt[2];
  real_T Product1;                     // '<S5>/Product1'
  real_T output_speed;                 // '<S5>/STOP1'
  real_T rateLimiterRate;
  real_T minDistance;
  real_T Switch2;                      // '<S5>/Switch2'
  real_T Switch_a;                     // '<S5>/Switch'
  real_T Switch1;                      // '<S5>/Switch1'
  real_T RateLimiter;                  // '<S5>/Rate Limiter'
  real_T alpha;
  real_T v12;
  real_T v12_g;
  SL_Bus_std_msgs_Int32 In1_m;         // '<S27>/In1'
  SL_Bus_std_msgs_Int32 In1_e;         // '<S19>/In1'
  SL_Bus_std_msgs_Int32 In1_h;         // '<S18>/In1'
  SL_Bus_std_msgs_Int32 In1_d;         // '<S16>/In1'
  SL_Bus_std_msgs_Float64 r;
};

// Block states (default storage) for system '<Root>'
struct DW_KDAS_mat_pp_waypoints_T {
  nav_slalgs_internal_PurePursu_T obj; // '<S5>/Pure Pursuit'
  ros_slros2_internal_block_Pub_T obj_c;// '<S23>/SinkBlock'
  ros_slros2_internal_block_Pub_T obj_m;// '<S7>/SinkBlock'
  ros_slros2_internal_block_Sub_T obj_me;// '<S26>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_n;// '<S4>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_g;// '<S15>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_ng;// '<S14>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_p;// '<S13>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_l;// '<S12>/SourceBlock'
  ros_slros2_internal_block_Sub_T obj_f;// '<S11>/SourceBlock'
  real_T UnitDelay_DSTATE[128];        // '<S3>/Unit Delay'
  real_T PrevY;                        // '<S5>/Rate Limiter'
  uint8_T is_active_c23_KDAS_mat_pp_waypo;// '<S5>/STOP1'
  uint8_T is_c23_KDAS_mat_pp_waypoints;// '<S5>/STOP1'
  boolean_T triggered;                 // '<Root>/MATLAB Function'
};

// Real-time Model Data Structure
struct tag_RTM_KDAS_mat_pp_waypoints_T {
  const char_T * volatile errorStatus;
  const char_T* getErrorStatus() const;
  void setErrorStatus(const char_T* const volatile aErrorStatus);
};

// Class declaration for model KDAS_mat_pp_waypoints
class KDAS_mat_pp_waypoints
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL_KDAS_mat_pp_waypoint_T * getRTM();

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  KDAS_mat_pp_waypoints();

  // Destructor
  ~KDAS_mat_pp_waypoints();

  // private data and function members
 private:
  // Block signals
  B_KDAS_mat_pp_waypoints_T KDAS_mat_pp_waypoints_B;

  // Block states
  DW_KDAS_mat_pp_waypoints_T KDAS_mat_pp_waypoints_DW;

  // private member function(s) for subsystem '<Root>'
  real_T KDAS_mat_pp_waypoints_norm(const real_T x[2]);
  real_T KDAS_mat_pp__closestPointOnLine(const real_T pt1[2], real_T pt2[2],
    const real_T refPt[2]);
  void KDAS_mat__Publisher_setupImpl_i(const ros_slros2_internal_block_Pub_T
    *obj);
  void KDA_Subscriber_setupImpl_ihwjxj(const ros_slros2_internal_block_Sub_T
    *obj);
  void KDAS_mat_Subscriber_setupImpl_i(const ros_slros2_internal_block_Sub_T
    *obj);
  void KDAS_ma_Subscriber_setupImpl_ih(const ros_slros2_internal_block_Sub_T
    *obj);
  void KDAS_m_Subscriber_setupImpl_ihw(const ros_slros2_internal_block_Sub_T
    *obj);
  void KDAS__Subscriber_setupImpl_ihwj(const ros_slros2_internal_block_Sub_T
    *obj);
  void KDAS_mat_p_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T
    *obj);
  void KDAS_Subscriber_setupImpl_ihwjx(const ros_slros2_internal_block_Sub_T
    *obj);
  void KDAS_mat_pp_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
    *obj);

  // Real-Time Model
  RT_MODEL_KDAS_mat_pp_waypoint_T KDAS_mat_pp_waypoints_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S3>/Scope' : Unused code path elimination
//  Block '<S3>/Scope1' : Unused code path elimination
//  Block '<S3>/Scope2' : Unused code path elimination
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<Root>/Scope1' : Unused code path elimination
//  Block '<Root>/Scope10' : Unused code path elimination
//  Block '<Root>/Scope2' : Unused code path elimination
//  Block '<Root>/Scope3' : Unused code path elimination
//  Block '<Root>/Scope6' : Unused code path elimination
//  Block '<Root>/Scope8' : Unused code path elimination
//  Block '<Root>/Scope9' : Unused code path elimination
//  Block '<S5>/Scope' : Unused code path elimination
//  Block '<S5>/Scope1' : Unused code path elimination
//  Block '<S5>/Scope10' : Unused code path elimination
//  Block '<S5>/Scope11' : Unused code path elimination
//  Block '<S5>/Scope2' : Unused code path elimination
//  Block '<S5>/Scope3' : Unused code path elimination
//  Block '<S5>/Scope4' : Unused code path elimination
//  Block '<S5>/Scope5' : Unused code path elimination
//  Block '<S5>/Scope6' : Unused code path elimination
//  Block '<S5>/Scope7' : Unused code path elimination
//  Block '<S5>/Scope8' : Unused code path elimination
//  Block '<S5>/Scope9' : Unused code path elimination
//  Block '<S5>/Zero-Order Hold' : Eliminated since input and output rates are identical


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'KDAS_mat_pp_waypoints'
//  '<S1>'   : 'KDAS_mat_pp_waypoints/MATLAB Function'
//  '<S2>'   : 'KDAS_mat_pp_waypoints/ROS2 Command'
//  '<S3>'   : 'KDAS_mat_pp_waypoints/ROS2 Data Input'
//  '<S4>'   : 'KDAS_mat_pp_waypoints/Subscribe'
//  '<S5>'   : 'KDAS_mat_pp_waypoints/Subsystem'
//  '<S6>'   : 'KDAS_mat_pp_waypoints/ROS2 Command/Blank Message'
//  '<S7>'   : 'KDAS_mat_pp_waypoints/ROS2 Command/Publish'
//  '<S8>'   : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Compare To Constant'
//  '<S9>'   : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Compare To Constant1'
//  '<S10>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/MATLAB Function'
//  '<S11>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe'
//  '<S12>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe1'
//  '<S13>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe2'
//  '<S14>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe3'
//  '<S15>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/cartographer'
//  '<S16>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe/Enabled Subsystem'
//  '<S17>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe1/Enabled Subsystem'
//  '<S18>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe2/Enabled Subsystem'
//  '<S19>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe3/Enabled Subsystem'
//  '<S20>'  : 'KDAS_mat_pp_waypoints/ROS2 Data Input/cartographer/Enabled Subsystem'
//  '<S21>'  : 'KDAS_mat_pp_waypoints/Subscribe/Enabled Subsystem'
//  '<S22>'  : 'KDAS_mat_pp_waypoints/Subsystem/Blank Message'
//  '<S23>'  : 'KDAS_mat_pp_waypoints/Subsystem/Publish'
//  '<S24>'  : 'KDAS_mat_pp_waypoints/Subsystem/STOP1'
//  '<S25>'  : 'KDAS_mat_pp_waypoints/Subsystem/Subsystem2'
//  '<S26>'  : 'KDAS_mat_pp_waypoints/Subsystem/Subsystem2/stopsign'
//  '<S27>'  : 'KDAS_mat_pp_waypoints/Subsystem/Subsystem2/stopsign/Enabled Subsystem'

#endif                                 // KDAS_mat_pp_waypoints_h_

//
// File trailer for generated code.
//
// [EOF]
//
