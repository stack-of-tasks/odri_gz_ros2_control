cmake_minimum_required(VERSION 3.14)
set(CMAKE_VERBOSE_MAKEFILE True)

project(odri_gz_ros2_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Detects Ubuntu release
find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -c
OUTPUT_VARIABLE LSB_RELEASE_CODENAME_PPREFIX
OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Remove prefix Codename:
string(REGEX REPLACE "Codename:\t" "" LSB_RELEASE_CODENAME ${LSB_RELEASE_CODENAME_PPREFIX})
message(STATUS "LSB_RELEASE_CODENAME=${LSB_RELEASE_CODENAME}")

if(${LSB_RELEASE_CODENAME} STREQUAL "jammy")
  # If Jammy is detected uses Gazebo Fortress
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Fortress")
  find_package(ignition-plugin1 REQUIRED)
  set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
  set(GZ_PLUGIN ignition-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM ignition-gazebo${GZ_SIM_VER}::core)
elseif(${LSB_RELEASE_CODENAME} STREQUAL "noble")
  # If Jammy is detected uses Harmonic
  find_package(gz_sim_vendor)
  find_package(gz-sim)

  find_package(gz_plugin_vendor)
  find_package(gz-plugin)
endif()

include_directories(include)

add_library(${PROJECT_NAME}-system SHARED
  src/gz_ros2_control_plugin.cpp
)

if(${LSB_RELEASE_CODENAME} STREQUAL "jammy")
target_link_libraries(${PROJECT_NAME}-system
  ${GZ_SIM}
  ${GZ_PLUGIN}
)
elseif(${LSB_RELEASE_CODENAME} STREQUAL "noble")
message(STATUS "Add target ${PROJECT_NAME}-system using noble")
target_compile_definitions(${PROJECT_NAME}-system PUBLIC GZ_SIM_8)
target_link_libraries(${PROJECT_NAME}-system
  gz-sim::gz-sim
  gz-plugin::register
)
endif()

ament_target_dependencies(${PROJECT_NAME}-system
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  rclcpp_lifecycle
)

#########

add_library(gz_hardware_odri_plugins SHARED
  src/gz_system.cpp
)
ament_target_dependencies(gz_hardware_odri_plugins
  rclcpp_lifecycle
  hardware_interface
  rclcpp
)

if(${LSB_RELEASE_CODENAME} STREQUAL "jammy")
target_link_libraries(gz_hardware_odri_plugins
  ${GZ_SIM}
)
elseif(${LSB_RELEASE_CODENAME} STREQUAL "noble")
target_compile_definitions(gz_hardware_odri_plugins PUBLIC GZ_SIM_8)
target_link_libraries(gz_hardware_odri_plugins
  gz-sim::gz-sim
)
endif()

## Install
install(TARGETS
  gz_hardware_odri_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include
  DESTINATION include
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} gz_hardware_odri_plugins)

# Install directories
install(TARGETS ${PROJECT_NAME}-system
  DESTINATION lib
)

pluginlib_export_plugin_description_file(odri_gz_ros2_control gz_hardware_odri_plugins.xml)

# Setup the project
ament_package()
