cmake_minimum_required(VERSION 3.8)
project(dead_reckoning)

# --- 1. Compiler Settings ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# --- 2. Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# External Libraries
find_package(fast_gicp REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io registration filters)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED) 

# --- 3. Executable Targets ---

# Node: loopclosure_vgicp
add_executable(loopclosure_vgicp src/loopclosure_vgicp.cpp)
ament_target_dependencies(loopclosure_vgicp
  rclcpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs pcl_conversions pcl_ros
  fast_gicp GTSAM
)
target_include_directories(loopclosure_vgicp PUBLIC 
  ${GTSAM_INCLUDE_DIR} 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(loopclosure_vgicp 
  ${PCL_LIBRARIES} 
  gtsam
)

# Node: odom_mapping
add_executable(odom_mapping src/mapping.cpp)
ament_target_dependencies(odom_mapping
  rclcpp sensor_msgs nav_msgs pcl_conversions
)
target_include_directories(odom_mapping PUBLIC 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(odom_mapping 
  ${PCL_LIBRARIES}
)

# Node: gicp_odom
add_executable(gicp_odom src/gicp_odom.cpp)
ament_target_dependencies(gicp_odom
  rclcpp sensor_msgs nav_msgs pcl_conversions fast_gicp
)
target_include_directories(gicp_odom PUBLIC 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(gicp_odom 
  ${PCL_LIBRARIES}
)

# Node: fastgicp_odom
add_executable(fastgicp_odom src/fastgicp_odom.cpp)
ament_target_dependencies(fastgicp_odom
  rclcpp sensor_msgs nav_msgs pcl_conversions fast_gicp
)
target_include_directories(fastgicp_odom PUBLIC 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(fastgicp_odom 
  ${PCL_LIBRARIES}
)

# Node: fastvgicp_odom
add_executable(fastvgicp_odom src/fastvgicp_odom.cpp)
ament_target_dependencies(fastvgicp_odom
  rclcpp sensor_msgs nav_msgs pcl_conversions fast_gicp
)
target_include_directories(fastvgicp_odom PUBLIC 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(fastvgicp_odom 
  ${PCL_LIBRARIES}
)

# Node: imu_preintegration
add_executable(imu_preintegration src/imu_preintegration.cpp)
ament_target_dependencies(imu_preintegration
  rclcpp sensor_msgs nav_msgs geometry_msgs
  GTSAM
)
target_include_directories(imu_preintegration PUBLIC 
  ${GTSAM_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(imu_preintegration 
  gtsam
)


# NODE: mapping
add_executable(mapping src/mapping.cpp)
ament_target_dependencies(mapping
  rclcpp sensor_msgs nav_msgs pcl_conversions fast_gicp
)
target_include_directories(mapping PUBLIC 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(mapping 
  ${PCL_LIBRARIES}
)


# --- 4. Install Directives ---

# Install C++ Executables
install(TARGETS 
  loopclosure_vgicp
  odom_mapping
  gicp_odom
  fastgicp_odom
  fastvgicp_odom
  imu_preintegration
  mapping
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python Scripts
# install(PROGRAMS
#   ${PROJECT_NAME}/dvl_bridge.py
#   ${PROJECT_NAME}/imu_bridge.py
#   ${PROJECT_NAME}/depth_bridge.py
#   ${PROJECT_NAME}/ekf_to_csv_logger.py
#   ${PROJECT_NAME}/pub_icp_data.py
#   ${PROJECT_NAME}/pointcloud_intensity.py
#   ${PROJECT_NAME}/pointcloud_features.py
#   ${PROJECT_NAME}/sonar_noise.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(PROGRAMS ${PROJECT_NAME}/gt_map.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME gt_map)

install(PROGRAMS ${PROJECT_NAME}/dvl_bridge.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME dvl_bridge)

install(PROGRAMS ${PROJECT_NAME}/imu_bridge.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME imu_bridge)

install(PROGRAMS ${PROJECT_NAME}/depth_bridge.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME depth_bridge)

install(PROGRAMS ${PROJECT_NAME}/ekf_to_csv_logger.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME ekf_to_csv_logger)

install(PROGRAMS ${PROJECT_NAME}/pub_icp_data.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME pub_icp_data)

install(PROGRAMS ${PROJECT_NAME}/pointcloud_intensity.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME pointcloud_intensity)

install(PROGRAMS ${PROJECT_NAME}/pointcloud_features.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME pointcloud_features)

install(PROGRAMS ${PROJECT_NAME}/sonar_noise.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME sonar_noise)

install(PROGRAMS ${PROJECT_NAME}/odom_tf.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME odom_tf)

# Install Shared Directories
install(DIRECTORY launch config rviz description
  DESTINATION share/${PROJECT_NAME}
)

# --- 5. Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()