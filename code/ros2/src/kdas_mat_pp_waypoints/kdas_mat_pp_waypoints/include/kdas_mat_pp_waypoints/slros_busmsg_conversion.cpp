#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include "KDAS_mat_pp_waypoints_types.h"
#include "slros_msgconvert_utils.h"


[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Point& msgPtr, SL_Bus_geometry_msgs_Point const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Point* busPtr, const geometry_msgs::msg::Point& msgPtr);

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Float64& msgPtr, SL_Bus_std_msgs_Float64 const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Float64* busPtr, const std_msgs::msg::Float64& msgPtr);

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Float64MultiArray& msgPtr, SL_Bus_std_msgs_Float64MultiArray const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Float64MultiArray* busPtr, const std_msgs::msg::Float64MultiArray& msgPtr);

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Int32& msgPtr, SL_Bus_std_msgs_Int32 const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Int32* busPtr, const std_msgs::msg::Int32& msgPtr);

[[maybe_unused]] static void convertFromBus(std_msgs::msg::MultiArrayDimension& msgPtr, SL_Bus_std_msgs_MultiArrayDimension const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_MultiArrayDimension* busPtr, const std_msgs::msg::MultiArrayDimension& msgPtr);

[[maybe_unused]] static void convertFromBus(std_msgs::msg::MultiArrayLayout& msgPtr, SL_Bus_std_msgs_MultiArrayLayout const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_MultiArrayLayout* busPtr, const std_msgs::msg::MultiArrayLayout& msgPtr);



// Conversions between SL_Bus_geometry_msgs_Point and geometry_msgs::msg::Point

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Point& msgPtr, SL_Bus_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Point* busPtr, const geometry_msgs::msg::Point& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_std_msgs_Float64 and std_msgs::msg::Float64

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Float64& msgPtr, SL_Bus_std_msgs_Float64 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  msgPtr.data =  busPtr->data;
}

[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Float64* busPtr, const std_msgs::msg::Float64& msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  busPtr->data =  msgPtr.data;
}


// Conversions between SL_Bus_std_msgs_Float64MultiArray and std_msgs::msg::Float64MultiArray

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Float64MultiArray& msgPtr, SL_Bus_std_msgs_Float64MultiArray const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64MultiArray");

  convertFromBusVariablePrimitiveArray(msgPtr.data, busPtr->data, busPtr->data_SL_Info);
  convertFromBus(msgPtr.layout, &busPtr->layout);
}

[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Float64MultiArray* busPtr, const std_msgs::msg::Float64MultiArray& msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64MultiArray");

  convertToBusVariablePrimitiveArray(busPtr->data, busPtr->data_SL_Info, msgPtr.data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBus(&busPtr->layout, msgPtr.layout);
}


// Conversions between SL_Bus_std_msgs_Int32 and std_msgs::msg::Int32

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Int32& msgPtr, SL_Bus_std_msgs_Int32 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Int32");

  msgPtr.data =  busPtr->data;
}

[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Int32* busPtr, const std_msgs::msg::Int32& msgPtr)
{
  const std::string rosMessageType("std_msgs/Int32");

  busPtr->data =  msgPtr.data;
}


// Conversions between SL_Bus_std_msgs_MultiArrayDimension and std_msgs::msg::MultiArrayDimension

[[maybe_unused]] static void convertFromBus(std_msgs::msg::MultiArrayDimension& msgPtr, SL_Bus_std_msgs_MultiArrayDimension const* busPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayDimension");

  convertFromBusVariablePrimitiveArray(msgPtr.label, busPtr->label, busPtr->label_SL_Info);
  msgPtr.size =  busPtr->size;
  msgPtr.stride =  busPtr->stride;
}

[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_MultiArrayDimension* busPtr, const std_msgs::msg::MultiArrayDimension& msgPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayDimension");

  convertToBusVariablePrimitiveArray(busPtr->label, busPtr->label_SL_Info, msgPtr.label, slros::EnabledWarning(rosMessageType, "label"));
  busPtr->size =  msgPtr.size;
  busPtr->stride =  msgPtr.stride;
}


// Conversions between SL_Bus_std_msgs_MultiArrayLayout and std_msgs::msg::MultiArrayLayout

[[maybe_unused]] static void convertFromBus(std_msgs::msg::MultiArrayLayout& msgPtr, SL_Bus_std_msgs_MultiArrayLayout const* busPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayLayout");

  msgPtr.data_offset =  busPtr->data_offset;
  convertFromBusVariableNestedArray(msgPtr.dim, busPtr->dim, busPtr->dim_SL_Info);
}

[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_MultiArrayLayout* busPtr, const std_msgs::msg::MultiArrayLayout& msgPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayLayout");

  busPtr->data_offset =  msgPtr.data_offset;
  convertToBusVariableNestedArray(busPtr->dim, busPtr->dim_SL_Info, msgPtr.dim, slros::EnabledWarning(rosMessageType, "dim"));
}



#endif
