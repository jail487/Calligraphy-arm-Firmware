cmake_minimum_required(VERSION 3.8)
project(doghead_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/McuMotorTestCommand.msg"
  "msg/McuJointSetpoint.msg"
  "msg/McuState.msg"
  "msg/McuDiagnostics.msg"
  "msg/McuSessionControl.msg"
  "msg/EndEffectorReference.msg"
  "msg/DriverSessionState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_units test/test_joint_units.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../Firmware/doghead_arm/Application/Src/joint_units.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../Firmware/doghead_arm/Application/Src/motor_parameters.c"
  )
  target_include_directories(test_joint_units PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../Firmware/doghead_arm/Application/Inc"
  )
endif()

ament_package()

