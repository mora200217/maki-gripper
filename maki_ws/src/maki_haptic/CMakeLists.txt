cmake_minimum_required(VERSION 3.8)
project(maki_haptic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(maki_interfaces REQUIRED)

add_executable(haptic_server src/haptic_server.cpp)
ament_target_dependencies(haptic_server rclcpp std_msgs maki_interfaces)
target_link_libraries(haptic_server CURL::libcurl)

add_executable(haptic_client src/haptic_client.cpp)
ament_target_dependencies(haptic_client rclcpp std_msgs maki_interfaces)

install(TARGETS
  haptic_server
  haptic_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
