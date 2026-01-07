cmake_minimum_required(VERSION 3.16)
project(so101_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Build hardware interface plugin with C++ SCServo SDK
add_library(
  so101_hardware
  SHARED
  src/so101_system.cpp
  scservo_sdk/SCSerial.cpp
  scservo_sdk/SCS.cpp
  scservo_sdk/SMS_STS.cpp
  scservo_sdk/SCSCL.cpp
  scservo_sdk/SMSCL.cpp
  scservo_sdk/SMSBL.cpp
)

target_compile_features(so101_hardware PUBLIC cxx_std_17)
target_include_directories(so101_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/scservo_sdk>
  $<INSTALL_INTERFACE:include/so101_hardware>
)

ament_target_dependencies(
  so101_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/so101_hardware
)

install(TARGETS so101_hardware
  EXPORT export_so101_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## Export
ament_export_targets(export_so101_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
