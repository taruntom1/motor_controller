cmake_minimum_required(VERSION 3.8)
project(motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(
  include
)

# List of source files
set(SOURCE_FILES
  src/main.cpp
  src/ControlInterface.cpp
  src/print.cpp
  src/ReadJSON.cpp
  src/UARTProtocol.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Link dependencies
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_package()
