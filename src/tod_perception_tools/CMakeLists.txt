cmake_minimum_required(VERSION 3.8)
project(tod_perception_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(tod_dummy_perception_pub src/tod_dummy_perception_pub_node.cpp)
target_include_directories(tod_dummy_perception_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tod_dummy_perception_pub PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  tod_dummy_perception_pub
  "rclcpp"
  "autoware_perception_msgs"
  "geometry_msgs"
  "ament_index_cpp"
  "tf2"
  "tf2_geometry_msgs"
)
target_link_libraries(tod_dummy_perception_pub yaml-cpp)

add_executable(tod_detected_object_merger src/tod_detected_object_merger_node.cpp)
target_include_directories(tod_detected_object_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tod_detected_object_merger PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  tod_detected_object_merger
  "rclcpp"
  "autoware_perception_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

install(TARGETS
  tod_dummy_perception_pub
  tod_detected_object_merger
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
