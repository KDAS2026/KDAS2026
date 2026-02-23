//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: KDAS_mat_pp_waypoints.cpp
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
#include "KDAS_mat_pp_waypoints.h"
#include "KDAS_mat_pp_waypoints_types.h"
#include "rtwtypes.h"
#include <string.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include <math.h>
#include "KDAS_mat_pp_waypoints_private.h"
#include "rmw/qos_profiles.h"
#include <stddef.h>
#include "rt_defines.h"

// Named constants for Chart: '<S5>/STOP1'
const uint8_T KDAS_mat_pp_waypoin_IN_Stopping = 1U;
const uint8_T KDAS_mat_pp_waypoints_IN_nomal = 2U;
real_T KDAS_mat_pp_waypoints::KDAS_mat_pp_waypoints_norm(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  // Start for MATLABSystem: '<S5>/Pure Pursuit'
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  // Start for MATLABSystem: '<S5>/Pure Pursuit'
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);

  // Start for MATLABSystem: '<S5>/Pure Pursuit'
  if (rtIsNaN(y)) {
    int32_T c_k;
    c_k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (c_k < 2) {
        if (rtIsNaN(x[c_k])) {
          exitg1 = 1;
        } else {
          c_k++;
        }
      } else {
        y = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

real_T KDAS_mat_pp_waypoints::KDAS_mat_pp__closestPointOnLine(const real_T pt1[2],
  real_T pt2[2], const real_T refPt[2])
{
  real_T distance;
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;

  // Start for MATLABSystem: '<S5>/Pure Pursuit'
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(pt1[b_k] == pt2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (p) {
    pt2[0] = pt1[0];
    KDAS_mat_pp_waypoints_B.refPt[0] = refPt[0] - pt1[0];
    pt2[1] = pt1[1];
    KDAS_mat_pp_waypoints_B.refPt[1] = refPt[1] - pt1[1];
    distance = KDAS_mat_pp_waypoints_norm(KDAS_mat_pp_waypoints_B.refPt);
  } else {
    KDAS_mat_pp_waypoints_B.alpha = pt2[0] - pt1[0];
    KDAS_mat_pp_waypoints_B.v12 = (pt2[0] - refPt[0]) *
      KDAS_mat_pp_waypoints_B.alpha;
    KDAS_mat_pp_waypoints_B.v12_g = KDAS_mat_pp_waypoints_B.alpha *
      KDAS_mat_pp_waypoints_B.alpha;
    KDAS_mat_pp_waypoints_B.alpha = pt2[1] - pt1[1];
    KDAS_mat_pp_waypoints_B.alpha = ((pt2[1] - refPt[1]) *
      KDAS_mat_pp_waypoints_B.alpha + KDAS_mat_pp_waypoints_B.v12) /
      (KDAS_mat_pp_waypoints_B.alpha * KDAS_mat_pp_waypoints_B.alpha +
       KDAS_mat_pp_waypoints_B.v12_g);
    if (KDAS_mat_pp_waypoints_B.alpha > 1.0) {
      pt2[0] = pt1[0];
      pt2[1] = pt1[1];
    } else if (!(KDAS_mat_pp_waypoints_B.alpha < 0.0)) {
      pt2[0] = (1.0 - KDAS_mat_pp_waypoints_B.alpha) * pt2[0] +
        KDAS_mat_pp_waypoints_B.alpha * pt1[0];
      pt2[1] = (1.0 - KDAS_mat_pp_waypoints_B.alpha) * pt2[1] +
        KDAS_mat_pp_waypoints_B.alpha * pt1[1];
    }

    KDAS_mat_pp_waypoints_B.refPt[0] = refPt[0] - pt2[0];
    KDAS_mat_pp_waypoints_B.refPt[1] = refPt[1] - pt2[1];
    distance = KDAS_mat_pp_waypoints_norm(KDAS_mat_pp_waypoints_B.refPt);
  }

  // End of Start for MATLABSystem: '<S5>/Pure Pursuit'
  return distance;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void KDAS_mat_pp_waypoints::KDAS_mat__Publisher_setupImpl_i(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[7];
  static const char_T b_zeroDelimTopic_0[7] = "/scope";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S23>/SinkBlock'
  KDAS_mat_pp_waypoints_B.deadline_cv.sec = 0.0;
  KDAS_mat_pp_waypoints_B.deadline_cv.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 KDAS_mat_pp_waypoints_B.deadline_cv, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 7; i++) {
    // Start for MATLABSystem: '<S23>/SinkBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_KDAS_mat_pp_waypoints_740.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void KDAS_mat_pp_waypoints::KDA_Subscriber_setupImpl_ihwjxj(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[6];
  static const char_T b_zeroDelimTopic_0[6] = "/stop";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S26>/SourceBlock'
  KDAS_mat_pp_waypoints_B.deadline_g.sec = 0.0;
  KDAS_mat_pp_waypoints_B.deadline_g.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 KDAS_mat_pp_waypoints_B.deadline_g, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 6; i++) {
    // Start for MATLABSystem: '<S26>/SourceBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_KDAS_mat_pp_waypoints_379.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void KDAS_mat_pp_waypoints::KDAS_mat_Subscriber_setupImpl_i(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[14];
  static const char_T b_zeroDelimTopic_0[14] = "/pp_waypoints";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S12>/SourceBlock'
  KDAS_mat_pp_waypoints_B.deadline.sec = 0.0;
  KDAS_mat_pp_waypoints_B.deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 KDAS_mat_pp_waypoints_B.deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 14; i++) {
    // Start for MATLABSystem: '<S12>/SourceBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_KDAS_mat_pp_waypoints_770.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void KDAS_mat_pp_waypoints::KDAS_ma_Subscriber_setupImpl_ih(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[18] = "/pp_waypoints_len";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S13>/SourceBlock'
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 18; i++) {
    // Start for MATLABSystem: '<S13>/SourceBlock'
    KDAS_mat_pp_waypoints_B.b_zeroDelimTopic[i] = b_zeroDelimTopic[i];
  }

  Sub_KDAS_mat_pp_waypoints_775.createSubscriber
    (&KDAS_mat_pp_waypoints_B.b_zeroDelimTopic[0], qos_profile);
}

void KDAS_mat_pp_waypoints::KDAS_m_Subscriber_setupImpl_ihw(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[6];
  static const char_T b_zeroDelimTopic_0[6] = "/stop";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S14>/SourceBlock'
  KDAS_mat_pp_waypoints_B.deadline_f.sec = 0.0;
  KDAS_mat_pp_waypoints_B.deadline_f.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 KDAS_mat_pp_waypoints_B.deadline_f, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 6; i++) {
    // Start for MATLABSystem: '<S14>/SourceBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_KDAS_mat_pp_waypoints_782.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void KDAS_mat_pp_waypoints::KDAS__Subscriber_setupImpl_ihwj(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[10];
  static const char_T b_zeroDelimTopic_0[10] = "/location";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S15>/SourceBlock'
  KDAS_mat_pp_waypoints_B.deadline_p.sec = 0.0;
  KDAS_mat_pp_waypoints_B.deadline_p.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 KDAS_mat_pp_waypoints_B.deadline_p, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 10; i++) {
    // Start for MATLABSystem: '<S15>/SourceBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_KDAS_mat_pp_waypoints_254.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void KDAS_mat_pp_waypoints::KDAS_mat_p_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[11];
  static const char_T b_zeroDelimTopic_0[11] = "/path_mode";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S11>/SourceBlock'
  KDAS_mat_pp_waypoints_B.deadline_c.sec = 0.0;
  KDAS_mat_pp_waypoints_B.deadline_c.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 KDAS_mat_pp_waypoints_B.deadline_c, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 11; i++) {
    // Start for MATLABSystem: '<S11>/SourceBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_KDAS_mat_pp_waypoints_551.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void KDAS_mat_pp_waypoints::KDAS_Subscriber_setupImpl_ihwjx(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[10];
  static const char_T b_zeroDelimTopic_0[10] = "/sim_time";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S4>/SourceBlock'
  KDAS_mat_pp_waypoints_B.deadline_b.sec = 0.0;
  KDAS_mat_pp_waypoints_B.deadline_b.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 KDAS_mat_pp_waypoints_B.deadline_b, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 10; i++) {
    // Start for MATLABSystem: '<S4>/SourceBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_KDAS_mat_pp_waypoints_598.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void KDAS_mat_pp_waypoints::KDAS_mat_pp_Publisher_setupImpl(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[13];
  static const char_T b_zeroDelimTopic_0[13] = "/simulinkOut";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S7>/SinkBlock'
  KDAS_mat_pp_waypoints_B.deadline_k.sec = 0.0;
  KDAS_mat_pp_waypoints_B.deadline_k.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 KDAS_mat_pp_waypoints_B.deadline_k, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 13; i++) {
    // Start for MATLABSystem: '<S7>/SinkBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_KDAS_mat_pp_waypoints_314.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

// Model step function
void KDAS_mat_pp_waypoints::step()
{
  SL_Bus_std_msgs_Int32 rtb_SourceBlock_o2_c_0;
  int32_T i;
  int32_T n;
  int32_T qY;
  int32_T tmp;
  int32_T tmp_size_idx_0;
  boolean_T b_varargout_1;
  boolean_T exitg1;
  boolean_T searchFlag;

  // MATLABSystem: '<S12>/SourceBlock'
  searchFlag = Sub_KDAS_mat_pp_waypoints_770.getLatestMessage
    (&KDAS_mat_pp_waypoints_B.rtb_SourceBlock_o2_m);

  // MATLABSystem: '<S13>/SourceBlock'
  b_varargout_1 = Sub_KDAS_mat_pp_waypoints_775.getLatestMessage
    (&rtb_SourceBlock_o2_c_0);

  // Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S18>/Enable'

  // Start for MATLABSystem: '<S13>/SourceBlock'
  if (b_varargout_1) {
    // SignalConversion generated from: '<S18>/In1'
    KDAS_mat_pp_waypoints_B.In1_h = rtb_SourceBlock_o2_c_0;
  }

  // End of Start for MATLABSystem: '<S13>/SourceBlock'
  // End of Outputs for SubSystem: '<S13>/Enabled Subsystem'

  // MATLABSystem: '<S14>/SourceBlock'
  b_varargout_1 = Sub_KDAS_mat_pp_waypoints_782.getLatestMessage
    (&rtb_SourceBlock_o2_c_0);

  // Outputs for Enabled SubSystem: '<S14>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S19>/Enable'

  // Start for MATLABSystem: '<S14>/SourceBlock'
  if (b_varargout_1) {
    // SignalConversion generated from: '<S19>/In1'
    KDAS_mat_pp_waypoints_B.In1_e = rtb_SourceBlock_o2_c_0;
  }

  // End of Start for MATLABSystem: '<S14>/SourceBlock'
  // End of Outputs for SubSystem: '<S14>/Enabled Subsystem'

  // Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S17>/Enable'

  // Start for MATLABSystem: '<S12>/SourceBlock'
  if (searchFlag) {
    // SignalConversion generated from: '<S17>/In1'
    KDAS_mat_pp_waypoints_B.In1 = KDAS_mat_pp_waypoints_B.rtb_SourceBlock_o2_m;
  }

  // End of Outputs for SubSystem: '<S12>/Enabled Subsystem'

  // MATLAB Function: '<S3>/MATLAB Function' incorporates:
  //   SignalConversion generated from: '<S3>/Bus Selector1'
  //   SignalConversion generated from: '<S3>/Bus Selector'

  memset(&KDAS_mat_pp_waypoints_B.W[0], 0, sizeof(real_T) << 7U);
  n = KDAS_mat_pp_waypoints_B.In1_h.data;
  if (KDAS_mat_pp_waypoints_B.In1_h.data < 0) {
    n = 0;
  } else if (KDAS_mat_pp_waypoints_B.In1_h.data > 64) {
    n = 64;
  }

  for (i = 0; i < n; i++) {
    if (i + 1 < -2147483647) {
      qY = MIN_int32_T;
    } else {
      qY = i;
    }

    if (qY > 1073741823) {
      qY = MAX_int32_T;
    } else {
      if (qY <= -1073741824) {
        qY = MIN_int32_T;
      } else {
        qY <<= 1;
      }

      qY++;
    }

    KDAS_mat_pp_waypoints_B.W[i] = KDAS_mat_pp_waypoints_B.In1.data[qY - 1];
    if (qY > 2147483646) {
      qY = MAX_int32_T;
    } else {
      qY++;
    }

    KDAS_mat_pp_waypoints_B.W[i + 64] = KDAS_mat_pp_waypoints_B.In1.data[qY - 1];
  }

  // End of MATLAB Function: '<S3>/MATLAB Function'

  // Switch: '<S3>/Switch' incorporates:
  //   Constant: '<S8>/Constant'
  //   Constant: '<S9>/Constant'
  //   Logic: '<S3>/Logical Operator'
  //   MATLABSystem: '<S12>/SourceBlock'
  //   RelationalOperator: '<S8>/Compare'
  //   RelationalOperator: '<S9>/Compare'
  //   SignalConversion generated from: '<S3>/Bus Selector1'
  //   UnitDelay: '<S3>/Unit Delay'
  //
  if (searchFlag && (KDAS_mat_pp_waypoints_B.In1_h.data >= 10) &&
      (KDAS_mat_pp_waypoints_B.In1_e.data == 0)) {
    memcpy(&KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[0],
           &KDAS_mat_pp_waypoints_B.W[0], sizeof(real_T) << 7U);
  }

  // End of Switch: '<S3>/Switch'

  // MATLABSystem: '<S15>/SourceBlock'
  searchFlag = Sub_KDAS_mat_pp_waypoints_254.getLatestMessage
    (&KDAS_mat_pp_waypoints_B.BusAssignment);

  // Outputs for Enabled SubSystem: '<S15>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S20>/Enable'

  // Start for MATLABSystem: '<S15>/SourceBlock'
  if (searchFlag) {
    // SignalConversion generated from: '<S20>/In1'
    KDAS_mat_pp_waypoints_B.In1_g = KDAS_mat_pp_waypoints_B.BusAssignment;
  }

  // End of Start for MATLABSystem: '<S15>/SourceBlock'
  // End of Outputs for SubSystem: '<S15>/Enabled Subsystem'

  // BusAssignment: '<S2>/Bus Assignment'
  memset(&KDAS_mat_pp_waypoints_B.BusAssignment, 0, sizeof
         (SL_Bus_geometry_msgs_Point));

  // MATLABSystem: '<S11>/SourceBlock'
  searchFlag = Sub_KDAS_mat_pp_waypoints_551.getLatestMessage
    (&rtb_SourceBlock_o2_c_0);

  // Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S16>/Enable'

  // Start for MATLABSystem: '<S11>/SourceBlock'
  if (searchFlag) {
    // SignalConversion generated from: '<S16>/In1'
    KDAS_mat_pp_waypoints_B.In1_d = rtb_SourceBlock_o2_c_0;
  }

  // End of Start for MATLABSystem: '<S11>/SourceBlock'
  // End of Outputs for SubSystem: '<S11>/Enabled Subsystem'

  // MATLABSystem: '<S4>/SourceBlock'
  searchFlag = Sub_KDAS_mat_pp_waypoints_598.getLatestMessage
    (&KDAS_mat_pp_waypoints_B.r);

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   MATLABSystem: '<S26>/SourceBlock'
  //   MATLABSystem: '<S4>/SourceBlock'
  //
  KDAS_mat_pp_waypoints_DW.triggered = (searchFlag ||
    KDAS_mat_pp_waypoints_DW.triggered);

  // Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
  //   EnablePort: '<S5>/Enable'

  if (KDAS_mat_pp_waypoints_DW.triggered) {
    // SignalConversion generated from: '<S5>/Pure Pursuit' incorporates:
    //   SignalConversion generated from: '<Root>/Bus Selector'
    //
    KDAS_mat_pp_waypoints_B.TmpSignalConversionAtPurePu[0] =
      KDAS_mat_pp_waypoints_B.In1_g.x;
    KDAS_mat_pp_waypoints_B.TmpSignalConversionAtPurePu[1] =
      KDAS_mat_pp_waypoints_B.In1_g.y;
    KDAS_mat_pp_waypoints_B.TmpSignalConversionAtPurePu[2] =
      KDAS_mat_pp_waypoints_B.In1_g.z;

    // Switch: '<S5>/Switch2' incorporates:
    //   Constant: '<S5>/Constant'
    //   Constant: '<S5>/Constant2'
    //   Constant: '<S5>/Constant3'
    //   Constant: '<S5>/Constant4'
    //   SignalConversion generated from: '<S3>/Bus Selector2'
    //   Switch: '<S5>/Switch'

    if (KDAS_mat_pp_waypoints_B.In1_d.data > 0) {
      KDAS_mat_pp_waypoints_B.Switch2 = 0.4;
      KDAS_mat_pp_waypoints_B.Switch_a = 0.33;
    } else {
      KDAS_mat_pp_waypoints_B.Switch2 = 0.8;
      KDAS_mat_pp_waypoints_B.Switch_a = 0.8;
    }

    // End of Switch: '<S5>/Switch2'

    // MATLABSystem: '<S5>/Pure Pursuit' incorporates:
    //   SignalConversion generated from: '<Root>/Bus Selector'
    //   Switch: '<S3>/Switch'
    //   UnitDelay: '<S3>/Unit Delay'
    //
    if (KDAS_mat_pp_waypoints_DW.obj.MaxAngularVelocity != 0.5) {
      KDAS_mat_pp_waypoints_DW.obj.MaxAngularVelocity = 0.5;
    }

    if (!(KDAS_mat_pp_waypoints_DW.obj.DesiredLinearVelocity ==
          KDAS_mat_pp_waypoints_B.Switch2)) {
      KDAS_mat_pp_waypoints_DW.obj.DesiredLinearVelocity =
        KDAS_mat_pp_waypoints_B.Switch2;
    }

    if (!(KDAS_mat_pp_waypoints_DW.obj.LookaheadDistance ==
          KDAS_mat_pp_waypoints_B.Switch_a)) {
      KDAS_mat_pp_waypoints_DW.obj.LookaheadDistance =
        KDAS_mat_pp_waypoints_B.Switch_a;
    }

    searchFlag = false;
    b_varargout_1 = true;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 128)) {
      if ((KDAS_mat_pp_waypoints_DW.obj.WaypointsInternal[i] ==
           KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[i]) || (rtIsNaN
           (KDAS_mat_pp_waypoints_DW.obj.WaypointsInternal[i]) && rtIsNaN
           (KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[i]))) {
        i++;
      } else {
        b_varargout_1 = false;
        exitg1 = true;
      }
    }

    if (b_varargout_1) {
      searchFlag = true;
    }

    if (!searchFlag) {
      memcpy(&KDAS_mat_pp_waypoints_DW.obj.WaypointsInternal[0],
             &KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[0], sizeof(real_T) << 7U);
      KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex = 0.0;
    }

    for (qY = 0; qY < 128; qY++) {
      KDAS_mat_pp_waypoints_B.b[qY] = !rtIsNaN
        (KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[qY]);
    }

    qY = 0;
    for (i = 0; i < 64; i++) {
      searchFlag = (KDAS_mat_pp_waypoints_B.b[i] && KDAS_mat_pp_waypoints_B.b[i
                    + 64]);
      KDAS_mat_pp_waypoints_B.bv[i] = searchFlag;
      if (searchFlag) {
        qY++;
      }
    }

    tmp_size_idx_0 = qY;
    qY = 0;
    for (i = 0; i < 64; i++) {
      if (KDAS_mat_pp_waypoints_B.bv[i]) {
        KDAS_mat_pp_waypoints_B.tmp_data[qY] = static_cast<int8_T>(i);
        qY++;
      }
    }

    if (tmp_size_idx_0 == 0) {
      KDAS_mat_pp_waypoints_B.Switch2 = 0.0;
      KDAS_mat_pp_waypoints_B.Switch_a = 0.0;
    } else {
      searchFlag = false;
      if (KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex == 0.0) {
        searchFlag = true;
        KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[0] =
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[KDAS_mat_pp_waypoints_B.tmp_data
          [0]];
        KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[1] =
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[KDAS_mat_pp_waypoints_B.tmp_data
          [0] + 64];
        KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex = 1.0;
      }

      if (tmp_size_idx_0 == 1) {
        KDAS_mat_pp_waypoints_B.Switch_a =
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[KDAS_mat_pp_waypoints_B.tmp_data
          [0]];
        KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[0] =
          KDAS_mat_pp_waypoints_B.Switch_a;
        KDAS_mat_pp_waypoints_B.rateLimiterRate =
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[KDAS_mat_pp_waypoints_B.tmp_data
          [0] + 64];
        KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[1] =
          KDAS_mat_pp_waypoints_B.rateLimiterRate;
        KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[0] =
          KDAS_mat_pp_waypoints_B.Switch_a;
        KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[1] =
          KDAS_mat_pp_waypoints_B.rateLimiterRate;
      } else {
        i = KDAS_mat_pp_waypoints_B.tmp_data[static_cast<int32_T>
          (KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex + 1.0) - 1];
        KDAS_mat_pp_waypoints_B.lookaheadStartPt[0] =
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[i];
        KDAS_mat_pp_waypoints_B.lookaheadStartPt[1] =
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[i + 64];
        KDAS_mat_pp_waypoints_B.minDistance = KDAS_mat_pp__closestPointOnLine
          (KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint,
           KDAS_mat_pp_waypoints_B.lookaheadStartPt,
           &KDAS_mat_pp_waypoints_B.TmpSignalConversionAtPurePu[0]);
        KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[0] =
          KDAS_mat_pp_waypoints_B.lookaheadStartPt[0];
        KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[0] =
          KDAS_mat_pp_waypoints_B.lookaheadStartPt[0] -
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[KDAS_mat_pp_waypoints_B.tmp_data
          [static_cast<int32_T>(KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex
          + 1.0) - 1]];
        KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[1] =
          KDAS_mat_pp_waypoints_B.lookaheadStartPt[1];
        KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[1] =
          KDAS_mat_pp_waypoints_B.lookaheadStartPt[1] -
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[KDAS_mat_pp_waypoints_B.tmp_data
          [static_cast<int32_T>(KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex
          + 1.0) - 1] + 64];
        KDAS_mat_pp_waypoints_B.Switch2 = KDAS_mat_pp_waypoints_norm
          (KDAS_mat_pp_waypoints_B.lookaheadStartPt_c);
        KDAS_mat_pp_waypoints_B.Switch1 =
          KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex + 1.0;
        i = static_cast<int32_T>((1.0 -
          (KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex + 1.0)) + (
          static_cast<real_T>(tmp_size_idx_0) - 1.0)) - 1;
        n = 0;
        exitg1 = false;
        while ((!exitg1) && (n <= i)) {
          KDAS_mat_pp_waypoints_B.RateLimiter = KDAS_mat_pp_waypoints_B.Switch1
            + static_cast<real_T>(n);
          if ((!searchFlag) && (KDAS_mat_pp_waypoints_B.Switch2 >
                                KDAS_mat_pp_waypoints_DW.obj.LookaheadDistance))
          {
            exitg1 = true;
          } else {
            qY = KDAS_mat_pp_waypoints_B.tmp_data[static_cast<int32_T>
              (KDAS_mat_pp_waypoints_B.RateLimiter + 1.0) - 1];
            KDAS_mat_pp_waypoints_B.Switch_a =
              KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[qY];
            tmp = KDAS_mat_pp_waypoints_B.tmp_data[static_cast<int32_T>
              (KDAS_mat_pp_waypoints_B.RateLimiter) - 1];
            KDAS_mat_pp_waypoints_B.rateLimiterRate =
              KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[tmp];
            KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[0] =
              KDAS_mat_pp_waypoints_B.rateLimiterRate -
              KDAS_mat_pp_waypoints_B.Switch_a;
            KDAS_mat_pp_waypoints_B.lookaheadStartPt[0] =
              KDAS_mat_pp_waypoints_B.Switch_a;
            KDAS_mat_pp_waypoints_B.dv[0] =
              KDAS_mat_pp_waypoints_B.rateLimiterRate;
            KDAS_mat_pp_waypoints_B.Switch_a =
              KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[qY + 64];
            KDAS_mat_pp_waypoints_B.rateLimiterRate =
              KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[tmp + 64];
            KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[1] =
              KDAS_mat_pp_waypoints_B.rateLimiterRate -
              KDAS_mat_pp_waypoints_B.Switch_a;
            KDAS_mat_pp_waypoints_B.lookaheadStartPt[1] =
              KDAS_mat_pp_waypoints_B.Switch_a;
            KDAS_mat_pp_waypoints_B.dv[1] =
              KDAS_mat_pp_waypoints_B.rateLimiterRate;
            KDAS_mat_pp_waypoints_B.Switch2 += KDAS_mat_pp_waypoints_norm
              (KDAS_mat_pp_waypoints_B.lookaheadStartPt_c);
            KDAS_mat_pp_waypoints_B.Switch_a = KDAS_mat_pp__closestPointOnLine
              (KDAS_mat_pp_waypoints_B.dv,
               KDAS_mat_pp_waypoints_B.lookaheadStartPt,
               &KDAS_mat_pp_waypoints_B.TmpSignalConversionAtPurePu[0]);
            if (KDAS_mat_pp_waypoints_B.Switch_a <
                KDAS_mat_pp_waypoints_B.minDistance) {
              KDAS_mat_pp_waypoints_B.minDistance =
                KDAS_mat_pp_waypoints_B.Switch_a;
              KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[0] =
                KDAS_mat_pp_waypoints_B.lookaheadStartPt[0];
              KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[1] =
                KDAS_mat_pp_waypoints_B.lookaheadStartPt[1];
              KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex =
                KDAS_mat_pp_waypoints_B.RateLimiter;
            }

            n++;
          }
        }

        qY = KDAS_mat_pp_waypoints_B.tmp_data[static_cast<int32_T>
          (KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex + 1.0) - 1];
        KDAS_mat_pp_waypoints_B.Switch_a =
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[qY];
        KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[0] =
          KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[0] -
          KDAS_mat_pp_waypoints_B.Switch_a;
        KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[1] =
          KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[1] -
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[qY + 64];
        KDAS_mat_pp_waypoints_B.Switch2 = KDAS_mat_pp_waypoints_norm
          (KDAS_mat_pp_waypoints_B.lookaheadStartPt_c);
        KDAS_mat_pp_waypoints_B.lookaheadStartPt[0] =
          KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[0];
        KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[0] =
          KDAS_mat_pp_waypoints_B.Switch_a;
        KDAS_mat_pp_waypoints_B.lookaheadStartPt[1] =
          KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[1];
        KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[1] =
          KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[KDAS_mat_pp_waypoints_B.tmp_data
          [static_cast<int32_T>(KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex
          + 1.0) - 1] + 64];
        KDAS_mat_pp_waypoints_B.Switch_a = KDAS_mat_pp_waypoints_B.Switch2 -
          KDAS_mat_pp_waypoints_DW.obj.LookaheadDistance;
        KDAS_mat_pp_waypoints_B.minDistance =
          KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex;
        while ((KDAS_mat_pp_waypoints_B.Switch_a < 0.0) &&
               (KDAS_mat_pp_waypoints_B.minDistance < static_cast<real_T>
                (tmp_size_idx_0) - 1.0)) {
          KDAS_mat_pp_waypoints_B.minDistance++;
          i = KDAS_mat_pp_waypoints_B.tmp_data[static_cast<int32_T>
            (KDAS_mat_pp_waypoints_B.minDistance) - 1];
          KDAS_mat_pp_waypoints_B.Switch1 =
            KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[i];
          KDAS_mat_pp_waypoints_B.lookaheadStartPt[0] =
            KDAS_mat_pp_waypoints_B.Switch1;
          qY = KDAS_mat_pp_waypoints_B.tmp_data[static_cast<int32_T>
            (KDAS_mat_pp_waypoints_B.minDistance + 1.0) - 1];
          KDAS_mat_pp_waypoints_B.Switch_a =
            KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[qY];
          KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[0] =
            KDAS_mat_pp_waypoints_B.Switch_a;
          KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[0] =
            KDAS_mat_pp_waypoints_B.Switch1 - KDAS_mat_pp_waypoints_B.Switch_a;
          KDAS_mat_pp_waypoints_B.Switch1 =
            KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[i + 64];
          KDAS_mat_pp_waypoints_B.lookaheadStartPt[1] =
            KDAS_mat_pp_waypoints_B.Switch1;
          KDAS_mat_pp_waypoints_B.Switch_a =
            KDAS_mat_pp_waypoints_DW.UnitDelay_DSTATE[qY + 64];
          KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[1] =
            KDAS_mat_pp_waypoints_B.Switch_a;
          KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[1] =
            KDAS_mat_pp_waypoints_B.Switch1 - KDAS_mat_pp_waypoints_B.Switch_a;
          KDAS_mat_pp_waypoints_B.Switch2 += KDAS_mat_pp_waypoints_norm
            (KDAS_mat_pp_waypoints_B.lookaheadStartPt_c);
          KDAS_mat_pp_waypoints_B.Switch_a = KDAS_mat_pp_waypoints_B.Switch2 -
            KDAS_mat_pp_waypoints_DW.obj.LookaheadDistance;
        }

        KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[0] =
          KDAS_mat_pp_waypoints_B.lookaheadStartPt[0] -
          KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[0];
        KDAS_mat_pp_waypoints_B.lookaheadStartPt_c[1] =
          KDAS_mat_pp_waypoints_B.lookaheadStartPt[1] -
          KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[1];
        KDAS_mat_pp_waypoints_B.Switch2 = KDAS_mat_pp_waypoints_B.Switch_a /
          KDAS_mat_pp_waypoints_norm(KDAS_mat_pp_waypoints_B.lookaheadStartPt_c);
        if (KDAS_mat_pp_waypoints_B.Switch2 > 0.0) {
          KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[0] = (1.0 -
            KDAS_mat_pp_waypoints_B.Switch2) *
            KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[0] +
            KDAS_mat_pp_waypoints_B.Switch2 *
            KDAS_mat_pp_waypoints_B.lookaheadStartPt[0];
          KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[1] = (1.0 -
            KDAS_mat_pp_waypoints_B.Switch2) *
            KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[1] +
            KDAS_mat_pp_waypoints_B.Switch2 *
            KDAS_mat_pp_waypoints_B.lookaheadStartPt[1];
        }
      }

      KDAS_mat_pp_waypoints_B.Switch2 = rt_atan2d_snf
        (KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[1] -
         KDAS_mat_pp_waypoints_B.In1_g.y,
         KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[0] -
         KDAS_mat_pp_waypoints_B.In1_g.x) - KDAS_mat_pp_waypoints_B.In1_g.z;
      if (fabs(KDAS_mat_pp_waypoints_B.Switch2) > 3.1415926535897931) {
        if (rtIsNaN(KDAS_mat_pp_waypoints_B.Switch2 + 3.1415926535897931) ||
            rtIsInf(KDAS_mat_pp_waypoints_B.Switch2 + 3.1415926535897931)) {
          KDAS_mat_pp_waypoints_B.Switch_a = (rtNaN);
        } else {
          KDAS_mat_pp_waypoints_B.Switch_a = fabs
            ((KDAS_mat_pp_waypoints_B.Switch2 + 3.1415926535897931) /
             6.2831853071795862);
          if (fabs(KDAS_mat_pp_waypoints_B.Switch_a - floor
                   (KDAS_mat_pp_waypoints_B.Switch_a + 0.5)) >
              2.2204460492503131E-16 * KDAS_mat_pp_waypoints_B.Switch_a) {
            KDAS_mat_pp_waypoints_B.Switch_a = fmod
              (KDAS_mat_pp_waypoints_B.Switch2 + 3.1415926535897931,
               6.2831853071795862);
          } else {
            KDAS_mat_pp_waypoints_B.Switch_a = 0.0;
          }

          if (KDAS_mat_pp_waypoints_B.Switch_a == 0.0) {
            KDAS_mat_pp_waypoints_B.Switch_a = 0.0;
          } else if (KDAS_mat_pp_waypoints_B.Switch_a < 0.0) {
            KDAS_mat_pp_waypoints_B.Switch_a += 6.2831853071795862;
          }
        }

        if ((KDAS_mat_pp_waypoints_B.Switch_a == 0.0) &&
            (KDAS_mat_pp_waypoints_B.Switch2 + 3.1415926535897931 > 0.0)) {
          KDAS_mat_pp_waypoints_B.Switch_a = 6.2831853071795862;
        }

        KDAS_mat_pp_waypoints_B.Switch2 = KDAS_mat_pp_waypoints_B.Switch_a -
          3.1415926535897931;
      }

      KDAS_mat_pp_waypoints_B.Switch_a = 2.0 * sin
        (KDAS_mat_pp_waypoints_B.Switch2) /
        KDAS_mat_pp_waypoints_DW.obj.LookaheadDistance;
      if (rtIsNaN(KDAS_mat_pp_waypoints_B.Switch_a)) {
        KDAS_mat_pp_waypoints_B.Switch_a = 0.0;
      }

      if (fabs(fabs(KDAS_mat_pp_waypoints_B.Switch2) - 3.1415926535897931) <
          1.4901161193847656E-8) {
        if (rtIsNaN(KDAS_mat_pp_waypoints_B.Switch_a)) {
          KDAS_mat_pp_waypoints_B.Switch_a = (rtNaN);
        } else if (KDAS_mat_pp_waypoints_B.Switch_a < 0.0) {
          KDAS_mat_pp_waypoints_B.Switch_a = -1.0;
        } else {
          KDAS_mat_pp_waypoints_B.Switch_a = (KDAS_mat_pp_waypoints_B.Switch_a >
            0.0);
        }
      }

      if (fabs(KDAS_mat_pp_waypoints_B.Switch_a) >
          KDAS_mat_pp_waypoints_DW.obj.MaxAngularVelocity) {
        if (rtIsNaN(KDAS_mat_pp_waypoints_B.Switch_a)) {
          KDAS_mat_pp_waypoints_B.Switch_a = (rtNaN);
        } else if (KDAS_mat_pp_waypoints_B.Switch_a < 0.0) {
          KDAS_mat_pp_waypoints_B.Switch_a = -1.0;
        } else {
          KDAS_mat_pp_waypoints_B.Switch_a = (KDAS_mat_pp_waypoints_B.Switch_a >
            0.0);
        }

        KDAS_mat_pp_waypoints_B.Switch_a *=
          KDAS_mat_pp_waypoints_DW.obj.MaxAngularVelocity;
      }

      KDAS_mat_pp_waypoints_B.Switch2 =
        KDAS_mat_pp_waypoints_DW.obj.DesiredLinearVelocity;
      KDAS_mat_pp_waypoints_DW.obj.LastPose[0] = KDAS_mat_pp_waypoints_B.In1_g.x;
      KDAS_mat_pp_waypoints_DW.obj.LastPose[1] = KDAS_mat_pp_waypoints_B.In1_g.y;
      KDAS_mat_pp_waypoints_DW.obj.LastPose[2] = KDAS_mat_pp_waypoints_B.In1_g.z;
    }

    // Trigonometry: '<S5>/Atan' incorporates:
    //   Constant: '<S5>/Constant1'
    //   MATLABSystem: '<S5>/Pure Pursuit'
    //   Product: '<S5>/Divide'
    //   Product: '<S5>/Product'

    KDAS_mat_pp_waypoints_B.Switch_a = atan(0.26 *
      KDAS_mat_pp_waypoints_B.Switch_a / KDAS_mat_pp_waypoints_B.Switch2);

    // Saturate: '<S5>/Saturation1'
    if (KDAS_mat_pp_waypoints_B.Switch_a > 0.35) {
      KDAS_mat_pp_waypoints_B.minDistance = 0.35;
    } else if (KDAS_mat_pp_waypoints_B.Switch_a < -0.35) {
      KDAS_mat_pp_waypoints_B.minDistance = -0.35;
    } else {
      KDAS_mat_pp_waypoints_B.minDistance = KDAS_mat_pp_waypoints_B.Switch_a;
    }

    // End of Saturate: '<S5>/Saturation1'

    // Switch: '<S5>/Switch1' incorporates:
    //   Constant: '<S5>/Constant5'
    //   Constant: '<S5>/Constant6'
    //   Saturate: '<S5>/Saturation'
    //   SignalConversion generated from: '<S3>/Bus Selector2'
    //   Switch: '<S5>/Switch3'

    if (KDAS_mat_pp_waypoints_B.In1_d.data > 0) {
      KDAS_mat_pp_waypoints_B.Switch1 = KDAS_mat_pp_waypoints_B.minDistance;
      KDAS_mat_pp_waypoints_B.RateLimiter = 0.77;//curve
    } else {
      if (KDAS_mat_pp_waypoints_B.Switch_a > 0.35) {
        // Saturate: '<S5>/Saturation'
        KDAS_mat_pp_waypoints_B.Switch1 = 0.35;
      } else if (KDAS_mat_pp_waypoints_B.Switch_a < -0.35) {
        // Saturate: '<S5>/Saturation'
        KDAS_mat_pp_waypoints_B.Switch1 = -0.35;
      } else {
        // Saturate: '<S5>/Saturation'
        KDAS_mat_pp_waypoints_B.Switch1 = KDAS_mat_pp_waypoints_B.Switch_a;
      }

      KDAS_mat_pp_waypoints_B.RateLimiter = 0.55;//straight
    }

    // End of Switch: '<S5>/Switch1'

    // RateLimiter: '<S5>/Rate Limiter'
    KDAS_mat_pp_waypoints_B.rateLimiterRate =
      KDAS_mat_pp_waypoints_B.RateLimiter - KDAS_mat_pp_waypoints_DW.PrevY;
    if (KDAS_mat_pp_waypoints_B.rateLimiterRate > 0.012) {
      KDAS_mat_pp_waypoints_B.RateLimiter = KDAS_mat_pp_waypoints_DW.PrevY +
        0.012;
    } else if (KDAS_mat_pp_waypoints_B.rateLimiterRate < -0.01) {
      KDAS_mat_pp_waypoints_B.RateLimiter = KDAS_mat_pp_waypoints_DW.PrevY -
        0.01;
    }

    KDAS_mat_pp_waypoints_DW.PrevY = KDAS_mat_pp_waypoints_B.RateLimiter;

    // End of RateLimiter: '<S5>/Rate Limiter'

    // Product: '<S5>/Product1'
    KDAS_mat_pp_waypoints_B.Product1 = KDAS_mat_pp_waypoints_B.RateLimiter *
      KDAS_mat_pp_waypoints_B.Switch1;

    // BusAssignment: '<S5>/Bus Assignment'
    KDAS_mat_pp_waypoints_B.BusAssignment_d.x = KDAS_mat_pp_waypoints_B.Switch_a;
    KDAS_mat_pp_waypoints_B.BusAssignment_d.y = KDAS_mat_pp_waypoints_B.Switch_a;
    KDAS_mat_pp_waypoints_B.BusAssignment_d.z =
      KDAS_mat_pp_waypoints_B.minDistance;

    // MATLABSystem: '<S23>/SinkBlock'
    Pub_KDAS_mat_pp_waypoints_740.publish
      (&KDAS_mat_pp_waypoints_B.BusAssignment_d);

    // MATLABSystem: '<S26>/SourceBlock'
    searchFlag = Sub_KDAS_mat_pp_waypoints_379.getLatestMessage
      (&rtb_SourceBlock_o2_c_0);

    // Outputs for Enabled SubSystem: '<S26>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S27>/Enable'

    if (searchFlag) {
      // SignalConversion generated from: '<S27>/In1'
      KDAS_mat_pp_waypoints_B.In1_m = rtb_SourceBlock_o2_c_0;
    }

    // End of Outputs for SubSystem: '<S26>/Enabled Subsystem'

    // Chart: '<S5>/STOP1' incorporates:
    //   MATLABSystem: '<S26>/SourceBlock'
    //   MATLABSystem: '<S5>/Pure Pursuit'
    //   SignalConversion generated from: '<S25>/Bus Selector'
    //
    if (KDAS_mat_pp_waypoints_DW.is_active_c23_KDAS_mat_pp_waypo == 0) {
      KDAS_mat_pp_waypoints_DW.is_active_c23_KDAS_mat_pp_waypo = 1U;
      KDAS_mat_pp_waypoints_DW.is_c23_KDAS_mat_pp_waypoints =
        KDAS_mat_pp_waypoints_IN_nomal;
    } else if (KDAS_mat_pp_waypoints_DW.is_c23_KDAS_mat_pp_waypoints ==
               KDAS_mat_pp_waypoin_IN_Stopping) {
      if (KDAS_mat_pp_waypoints_B.In1_m.data == 0) {
        KDAS_mat_pp_waypoints_DW.is_c23_KDAS_mat_pp_waypoints =
          KDAS_mat_pp_waypoints_IN_nomal;
      } else {
        KDAS_mat_pp_waypoints_B.output_speed = 0.0;
      }

      // case IN_nomal:
    } else if (KDAS_mat_pp_waypoints_B.In1_m.data == 1) {
      KDAS_mat_pp_waypoints_DW.is_c23_KDAS_mat_pp_waypoints =
        KDAS_mat_pp_waypoin_IN_Stopping;
      KDAS_mat_pp_waypoints_B.output_speed = 0.0;
    } else {
      KDAS_mat_pp_waypoints_B.output_speed = KDAS_mat_pp_waypoints_B.Switch2;
    }

    // End of Chart: '<S5>/STOP1'
  }

  // End of MATLAB Function: '<Root>/MATLAB Function'
  // End of Outputs for SubSystem: '<Root>/Subsystem'

  // BusAssignment: '<S2>/Bus Assignment'
  KDAS_mat_pp_waypoints_B.BusAssignment.x = KDAS_mat_pp_waypoints_B.output_speed;
  KDAS_mat_pp_waypoints_B.BusAssignment.y = KDAS_mat_pp_waypoints_B.Product1;

  // MATLABSystem: '<S7>/SinkBlock'
  Pub_KDAS_mat_pp_waypoints_314.publish(&KDAS_mat_pp_waypoints_B.BusAssignment);
}

// Model initialize function
void KDAS_mat_pp_waypoints::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    int32_T i;

    // SystemInitialize for Enabled SubSystem: '<Root>/Subsystem'
    // InitializeConditions for RateLimiter: '<S5>/Rate Limiter'
    KDAS_mat_pp_waypoints_DW.PrevY = 0.75;

    // Start for MATLABSystem: '<S5>/Pure Pursuit'
    KDAS_mat_pp_waypoints_DW.obj.MaxAngularVelocity = 0.5;
    KDAS_mat_pp_waypoints_DW.obj.isInitialized = 1;
    KDAS_mat_pp_waypoints_DW.obj.DesiredLinearVelocity = 0.0;
    KDAS_mat_pp_waypoints_DW.obj.LookaheadDistance = 0.0;
    for (i = 0; i < 128; i++) {
      KDAS_mat_pp_waypoints_DW.obj.WaypointsInternal[i] = (rtNaN);
    }

    // InitializeConditions for MATLABSystem: '<S5>/Pure Pursuit'
    KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[0] = 0.0;
    KDAS_mat_pp_waypoints_DW.obj.LookaheadPoint[1] = 0.0;
    KDAS_mat_pp_waypoints_DW.obj.LastPose[0] = 0.0;
    KDAS_mat_pp_waypoints_DW.obj.LastPose[1] = 0.0;
    KDAS_mat_pp_waypoints_DW.obj.LastPose[2] = 0.0;
    KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[0] = (rtNaN);
    KDAS_mat_pp_waypoints_DW.obj.ProjectionPoint[1] = (rtNaN);
    KDAS_mat_pp_waypoints_DW.obj.ProjectionLineIndex = 0.0;

    // Start for MATLABSystem: '<S23>/SinkBlock'
    KDAS_mat_pp_waypoints_DW.obj_c.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_c.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_c.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_c.isInitialized = 1;
    KDAS_mat__Publisher_setupImpl_i(&KDAS_mat_pp_waypoints_DW.obj_c);
    KDAS_mat_pp_waypoints_DW.obj_c.isSetupComplete = true;

    // Start for MATLABSystem: '<S26>/SourceBlock'
    KDAS_mat_pp_waypoints_DW.obj_me.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_me.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_me.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_me.isInitialized = 1;
    KDA_Subscriber_setupImpl_ihwjxj(&KDAS_mat_pp_waypoints_DW.obj_me);
    KDAS_mat_pp_waypoints_DW.obj_me.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<Root>/Subsystem'

    // Start for MATLABSystem: '<S12>/SourceBlock'
    KDAS_mat_pp_waypoints_DW.obj_l.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_l.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_l.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_l.isInitialized = 1;
    KDAS_mat_Subscriber_setupImpl_i(&KDAS_mat_pp_waypoints_DW.obj_l);
    KDAS_mat_pp_waypoints_DW.obj_l.isSetupComplete = true;

    // Start for MATLABSystem: '<S13>/SourceBlock'
    KDAS_mat_pp_waypoints_DW.obj_p.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_p.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_p.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_p.isInitialized = 1;
    KDAS_ma_Subscriber_setupImpl_ih(&KDAS_mat_pp_waypoints_DW.obj_p);
    KDAS_mat_pp_waypoints_DW.obj_p.isSetupComplete = true;

    // Start for MATLABSystem: '<S14>/SourceBlock'
    KDAS_mat_pp_waypoints_DW.obj_ng.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_ng.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_ng.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_ng.isInitialized = 1;
    KDAS_m_Subscriber_setupImpl_ihw(&KDAS_mat_pp_waypoints_DW.obj_ng);
    KDAS_mat_pp_waypoints_DW.obj_ng.isSetupComplete = true;

    // Start for MATLABSystem: '<S15>/SourceBlock'
    KDAS_mat_pp_waypoints_DW.obj_g.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_g.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_g.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_g.isInitialized = 1;
    KDAS__Subscriber_setupImpl_ihwj(&KDAS_mat_pp_waypoints_DW.obj_g);
    KDAS_mat_pp_waypoints_DW.obj_g.isSetupComplete = true;

    // Start for MATLABSystem: '<S11>/SourceBlock'
    KDAS_mat_pp_waypoints_DW.obj_f.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_f.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_f.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_f.isInitialized = 1;
    KDAS_mat_p_Subscriber_setupImpl(&KDAS_mat_pp_waypoints_DW.obj_f);
    KDAS_mat_pp_waypoints_DW.obj_f.isSetupComplete = true;

    // Start for MATLABSystem: '<S4>/SourceBlock'
    KDAS_mat_pp_waypoints_DW.obj_n.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_n.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_n.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_n.isInitialized = 1;
    KDAS_Subscriber_setupImpl_ihwjx(&KDAS_mat_pp_waypoints_DW.obj_n);
    KDAS_mat_pp_waypoints_DW.obj_n.isSetupComplete = true;

    // Start for MATLABSystem: '<S7>/SinkBlock'
    KDAS_mat_pp_waypoints_DW.obj_m.QOSAvoidROSNamespaceConventions = false;
    KDAS_mat_pp_waypoints_DW.obj_m.matlabCodegenIsDeleted = false;
    KDAS_mat_pp_waypoints_DW.obj_m.isSetupComplete = false;
    KDAS_mat_pp_waypoints_DW.obj_m.isInitialized = 1;
    KDAS_mat_pp_Publisher_setupImpl(&KDAS_mat_pp_waypoints_DW.obj_m);
    KDAS_mat_pp_waypoints_DW.obj_m.isSetupComplete = true;
  }
}

// Model terminate function
void KDAS_mat_pp_waypoints::terminate()
{
  // Terminate for MATLABSystem: '<S12>/SourceBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_l.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_l.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_l.isSetupComplete) {
      Sub_KDAS_mat_pp_waypoints_770.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S12>/SourceBlock'

  // Terminate for MATLABSystem: '<S13>/SourceBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_p.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_p.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_p.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_p.isSetupComplete) {
      Sub_KDAS_mat_pp_waypoints_775.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S13>/SourceBlock'

  // Terminate for MATLABSystem: '<S14>/SourceBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_ng.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_ng.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_ng.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_ng.isSetupComplete) {
      Sub_KDAS_mat_pp_waypoints_782.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S14>/SourceBlock'

  // Terminate for MATLABSystem: '<S15>/SourceBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_g.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_g.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_g.isSetupComplete) {
      Sub_KDAS_mat_pp_waypoints_254.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S15>/SourceBlock'

  // Terminate for MATLABSystem: '<S11>/SourceBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_f.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_f.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_f.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_f.isSetupComplete) {
      Sub_KDAS_mat_pp_waypoints_551.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S11>/SourceBlock'

  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_n.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_n.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_n.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_n.isSetupComplete) {
      Sub_KDAS_mat_pp_waypoints_598.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S4>/SourceBlock'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem'
  // Terminate for MATLABSystem: '<S23>/SinkBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_c.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_c.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_c.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_c.isSetupComplete) {
      Pub_KDAS_mat_pp_waypoints_740.resetPublisherPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S23>/SinkBlock'

  // Terminate for MATLABSystem: '<S26>/SourceBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_me.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_me.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_me.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_me.isSetupComplete) {
      Sub_KDAS_mat_pp_waypoints_379.resetSubscriberPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S26>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subsystem'

  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  if (!KDAS_mat_pp_waypoints_DW.obj_m.matlabCodegenIsDeleted) {
    KDAS_mat_pp_waypoints_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((KDAS_mat_pp_waypoints_DW.obj_m.isInitialized == 1) &&
        KDAS_mat_pp_waypoints_DW.obj_m.isSetupComplete) {
      Pub_KDAS_mat_pp_waypoints_314.resetPublisherPtr();//();
    }
  }

  // End of Terminate for MATLABSystem: '<S7>/SinkBlock'
}

// Constructor
KDAS_mat_pp_waypoints::KDAS_mat_pp_waypoints() :
  KDAS_mat_pp_waypoints_B(),
  KDAS_mat_pp_waypoints_DW(),
  KDAS_mat_pp_waypoints_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
KDAS_mat_pp_waypoints::~KDAS_mat_pp_waypoints()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_KDAS_mat_pp_waypoint_T * KDAS_mat_pp_waypoints::getRTM()
{
  return (&KDAS_mat_pp_waypoints_M);
}

const char_T* RT_MODEL_KDAS_mat_pp_waypoint_T::getErrorStatus() const
{
  return (errorStatus);
}

void RT_MODEL_KDAS_mat_pp_waypoint_T::setErrorStatus(const char_T* const
  volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

//
// File trailer for generated code.
//
// [EOF]
//
