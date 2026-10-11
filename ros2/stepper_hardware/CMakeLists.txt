cmake_minimum_required(VERSION 3.16)
project(stepper_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

add_library(stepper_hardware SHARED
  src/stepper_system.cpp
  src/serial_link.cpp
)
target_compile_features(stepper_hardware PUBLIC cxx_std_17)
target_include_directories(stepper_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/stepper_hardware>
)
target_link_libraries(stepper_hardware PUBLIC Threads::Threads)
ament_target_dependencies(stepper_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# บอก ros2_control ว่ามีปลั๊กอินตัวนี้อยู่
pluginlib_export_plugin_description_file(hardware_interface stepper_hardware.xml)

install(TARGETS stepper_hardware
  EXPORT export_stepper_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/stepper_hardware)
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_stepper_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
