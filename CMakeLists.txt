cmake_minimum_required(VERSION 3.8)
project(dummy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(alfa_node REQUIRED)
find_package(alfa_msg REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(dummy src/main.cpp src/LinkedList.cpp src/Node.cpp)
target_link_libraries(dummy ${PCL_LIBRARIES})
ament_target_dependencies(dummy rclcpp alfa_node sensor_msgs alfa_msg pcl_conversions)

install(TARGETS
  dummy
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
