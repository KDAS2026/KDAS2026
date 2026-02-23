// Copyright 2022-2025 The MathWorks, Inc.
// Generated 28-Jan-2026 20:19:37
#ifndef _SLROS2_INITIALIZE_H_
#define _SLROS2_INITIALIZE_H_
#include "KDAS_mat_pp_waypoints_types.h"
// Generic pub-sub header
#include "slros2_generic_pubsub.h"
#ifndef SET_QOS_VALUES
#define SET_QOS_VALUES(qosStruct, _history, _depth, _durability, _reliability, _deadline \
, _lifespan, _liveliness, _lease_duration, _avoid_ros_namespace_conventions)             \
    {                                                                                    \
        qosStruct.history = _history;                                                    \
        qosStruct.depth = _depth;                                                        \
        qosStruct.durability = _durability;                                              \
        qosStruct.reliability = _reliability;                                            \
        qosStruct.deadline.sec = _deadline.sec;                                          \
        qosStruct.deadline.nsec = _deadline.nsec;                                        \
        qosStruct.lifespan.sec = _lifespan.sec;                                          \
        qosStruct.lifespan.nsec = _lifespan.nsec;                                        \
        qosStruct.liveliness = _liveliness;                                              \
        qosStruct.liveliness_lease_duration.sec = _lease_duration.sec;                   \
        qosStruct.liveliness_lease_duration.nsec = _lease_duration.nsec;                 \
        qosStruct.avoid_ros_namespace_conventions = _avoid_ros_namespace_conventions;    \
    }
#endif
// Get QOS Settings from RMW
inline rclcpp::QoS getQOSSettingsFromRMW(const rmw_qos_profile_t& qosProfile) {
  // pull in history & depth
  auto init = rclcpp::QoSInitialization::from_rmw(qosProfile);
  // set durability, reliability, deadline, liveliness policy, liveliness lease duration and avoid ROS namespace convention
  rclcpp::QoS qos(init, qosProfile);
  return qos;
}
// KDAS_mat_pp_waypoints/ROS2 Command/Publish
extern SimulinkPublisher<geometry_msgs::msg::Point,SL_Bus_geometry_msgs_Point> Pub_KDAS_mat_pp_waypoints_314;
// KDAS_mat_pp_waypoints/Subsystem/Publish
extern SimulinkPublisher<geometry_msgs::msg::Point,SL_Bus_geometry_msgs_Point> Pub_KDAS_mat_pp_waypoints_740;
// KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe
extern SimulinkSubscriber<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Sub_KDAS_mat_pp_waypoints_551;
// KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe1
extern SimulinkSubscriber<std_msgs::msg::Float64MultiArray,SL_Bus_std_msgs_Float64MultiArray> Sub_KDAS_mat_pp_waypoints_770;
// KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe2
extern SimulinkSubscriber<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Sub_KDAS_mat_pp_waypoints_775;
// KDAS_mat_pp_waypoints/ROS2 Data Input/Subscribe3
extern SimulinkSubscriber<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Sub_KDAS_mat_pp_waypoints_782;
// KDAS_mat_pp_waypoints/ROS2 Data Input/cartographer
extern SimulinkSubscriber<geometry_msgs::msg::Point,SL_Bus_geometry_msgs_Point> Sub_KDAS_mat_pp_waypoints_254;
// KDAS_mat_pp_waypoints/Subscribe
extern SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_KDAS_mat_pp_waypoints_598;
// KDAS_mat_pp_waypoints/Subsystem/Subsystem2/stopsign
extern SimulinkSubscriber<std_msgs::msg::Int32,SL_Bus_std_msgs_Int32> Sub_KDAS_mat_pp_waypoints_379;
#endif
