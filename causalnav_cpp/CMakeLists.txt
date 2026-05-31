cmake_minimum_required(VERSION 3.8)
project(causalnav_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Use -O3 for maximum optimization. This is what makes C++ fast.
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 1. Find standard ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)

# OpenCV is required under the hood by image_geometry
find_package(OpenCV REQUIRED)

# ==========================================
# Node 1: Point Cloud Segmenter
# ==========================================
add_executable(pc_segmenter_node src/pc_segmenter_node.cpp)

# Link ROS2 dependencies to this node
ament_target_dependencies(pc_segmenter_node
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  yolo_msgs 
  message_filters
  tf2_ros 
  tf2_sensor_msgs 
  image_geometry
)
# Link standard C++ libraries (OpenCV)
target_include_directories(pc_segmenter_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pc_segmenter_node ${OpenCV_LIBRARIES})


# ==========================================
# Node 2: Dynamic Object Remover
# ==========================================
add_executable(dynamic_remover_node src/dynamic_remover_node.cpp)

ament_target_dependencies(dynamic_remover_node
  rclcpp 
  sensor_msgs 
  vision_msgs 
  message_filters
)

# ==========================================
# Install Directives
# ==========================================
# This tells ROS2 where to put your compiled executables so 'ros2 run' can find them
install(TARGETS
  pc_segmenter_node
  dynamic_remover_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
