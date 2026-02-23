cmake_minimum_required(VERSION 3.5)
project(kdas_mat_pp_waypoints)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Set CMAKE_BUILD_TYPE
set(CMAKE_BUILD_TYPE Release)
set(MW_TOP_LEVEL_PROJECT_NAME ${PROJECT_NAME})
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(ALIAS_ROS2_TF2)
  add_definitions(-Dtf2=ros2_tf2)
  add_definitions(-Dtf2_ros=ros2_tf2_ros)
  add_definitions(-Dmessage_filters=ros2_message_filters)
endif()
# Check the ROS 2 distribution and define a macro
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    message(STATUS "Building for ROS 2 Humble")
    add_definitions(-DROS2_DISTRO_HUMBLE)
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    message(STATUS "Building for ROS 2 Jazzy")
    add_definitions(-DROS2_DISTRO_JAZZY)
else()
    message(WARNING "ROS 2 distribution $ENV{ROS_DISTRO} is not recommended for this MATLAB version. Use 'ROS 2 Jazzy' for this MATLAB version.")
    message(WARNING "Building for ROS 2 Jazzy")
    add_definitions(-DROS2_DISTRO_JAZZY)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package( geometry_msgs REQUIRED)
find_package( rclcpp REQUIRED)
find_package( std_msgs REQUIRED)
# Add package include directories
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/kdas_mat_pp_waypoints")
set (CMAKE_SKIP_BUILD_RPATH false)
set (CMAKE_BUILD_WITH_INSTALL_RPATH true)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# Node
add_executable(KDAS_mat_pp_waypoints 
src/KDAS_mat_pp_waypoints.cpp
src/main.cpp
src/ros2nodeinterface.cpp
src/rtGetInf.cpp
src/rtGetNaN.cpp
src/rt_nonfinite.cpp
src/slros2_initialize.cpp
src/slros2_multi_threaded_executor.cpp
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
target_compile_options(KDAS_mat_pp_waypoints PUBLIC 
  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>:
  -Wall -Wextra -Wpedantic
  >)
endif()
target_compile_options(KDAS_mat_pp_waypoints PRIVATE
  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>:
  -DMODEL=KDAS_mat_pp_waypoints
  -DNUMST=1
  -DNCSTATES=0
  -DHAVESTDIO
  -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
  -DCLASSIC_INTERFACE=0
  -DALLOCATIONFCN=0
  -DTID01EQ=0
  -DTERMFCN=1
  -DONESTEPFCN=1
  -DMAT_FILE=0
  -DMULTI_INSTANCE_CODE=1
  -DINTEGER_CODE=0
  -DMT=0
  -DROS2_PROJECT
  -DSTACK_SIZE=64
  -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
  -DRT=RT
  -fpermissive  >
  )
ament_target_dependencies(
  KDAS_mat_pp_waypoints
  "geometry_msgs"
  "rclcpp"
  "std_msgs"
)
target_include_directories(KDAS_mat_pp_waypoints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(KDAS_mat_pp_waypoints 
	${CMAKE_DL_LIBS}
    )
install(TARGETS KDAS_mat_pp_waypoints
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(
  include
)
foreach(file_i ${CUDA_BINS})
add_custom_command(
                   TARGET KDAS_mat_pp_waypoints
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CMAKE_INSTALL_PREFIX}/KDAS_mat_pp_waypoints_ert_rtw/${file_i}
				   )
add_custom_command(
                   TARGET KDAS_mat_pp_waypoints
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CMAKE_INSTALL_PREFIX}/codegen/exe/KDAS_mat_pp_waypoints/${file_i}
				   )				   
endforeach(file_i)
# Launch 파일 및 리소스 설치 설정 추가
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
# Generated on Wed Jan 28 20:19:39 2026
