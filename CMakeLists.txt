cmake_minimum_required(VERSION 3.8)
project(sonar_localization)

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
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# External Libraries
find_package(fast_gicp REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io registration filters segmentation)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(small_gicp REQUIRED)

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

# Node: prior_map_localizer
add_executable(prior_map_localizer src/prior_map_localizer.cpp)
ament_target_dependencies(prior_map_localizer
  rclcpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs pcl_conversions pcl_ros
  fast_gicp tf2_ros GTSAM
)
target_include_directories(prior_map_localizer PUBLIC
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(prior_map_localizer
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


# Node: loopclosure_samll_vgicp (small_gicp version)
add_executable(loopclosure_samll_vgicp src/loopclosure_samll_vgicp.cpp)
ament_target_dependencies(loopclosure_samll_vgicp
  rclcpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs pcl_conversions pcl_ros
  GTSAM
)
target_include_directories(loopclosure_samll_vgicp PUBLIC
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(loopclosure_samll_vgicp
  ${PCL_LIBRARIES}
  gtsam
  small_gicp::small_gicp
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


# Node: prior_map_publisher
add_executable(prior_map_publisher src/prior_map_publisher.cpp)
ament_target_dependencies(prior_map_publisher
  rclcpp sensor_msgs pcl_conversions pcl_ros
)
target_include_directories(prior_map_publisher PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(prior_map_publisher
  ${PCL_LIBRARIES}
)

# Node: map_matcher (aligns SLAM map → global map, publishes /initialpose correction)
add_executable(map_matcher src/map_matcher.cpp)
ament_target_dependencies(map_matcher
  rclcpp sensor_msgs geometry_msgs pcl_conversions pcl_ros fast_gicp
)
target_include_directories(map_matcher PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(map_matcher ${PCL_LIBRARIES})

# Node: localization_prior (localization-only against a pre-built prior map)
add_executable(localization_prior src/localization_prior.cpp)
ament_target_dependencies(localization_prior
  rclcpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs pcl_conversions pcl_ros
  fast_gicp GTSAM tf2_ros tf2_eigen
)
target_include_directories(localization_prior PUBLIC
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(localization_prior
  ${PCL_LIBRARIES}
  gtsam)

# Node: localization_imu_pre (VGICP SLAM + IMU preintegration + DVL factor)
add_executable(localization_imu_pre src/localization_imu_pre.cpp)
ament_target_dependencies(localization_imu_pre
  rclcpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs pcl_conversions pcl_ros
  fast_gicp GTSAM tf2_ros
)
target_include_directories(localization_imu_pre PUBLIC
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(localization_imu_pre
  ${PCL_LIBRARIES}
  gtsam)
# --- 4. Install Directives ---

# Install C++ Executables
install(TARGETS
  map_matcher
  localization_prior
  localization_imu_pre
  loopclosure_vgicp
  loopclosure_samll_vgicp
  prior_map_localizer
  prior_map_publisher
  odom_mapping
  gicp_odom
  fastgicp_odom
  fastvgicp_odom
  mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS ${PROJECT_NAME}/gt_map.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME gt_map)

install(PROGRAMS ${PROJECT_NAME}/video.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME video)

install(PROGRAMS ${PROJECT_NAME}/image_comp.py 
        DESTINATION lib/${PROJECT_NAME} 
        RENAME image_comp)

install(PROGRAMS ${PROJECT_NAME}/nucleus_dvl_bridge.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME nucleus_dvl_bridge)

install(PROGRAMS ${PROJECT_NAME}/waterlinked_dvl_bridge.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME waterlinked_dvl_bridge)

install(PROGRAMS ${PROJECT_NAME}/sim_dvl_bridge.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME sim_dvl_bridge)

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

install(PROGRAMS ${PROJECT_NAME}/nucleus_imu_bridge.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME nucleus_imu_bridge)
        
install(PROGRAMS ${PROJECT_NAME}/camera_info_publisher.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME camera_info_publisher)
install(PROGRAMS ${PROJECT_NAME}/nucleus_ahrs_bridge.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME nucleus_ahrs_bridge)
install(PROGRAMS ${PROJECT_NAME}/camera2point.py
      DESTINATION lib/${PROJECT_NAME}
      RENAME camera2point)       

install(PROGRAMS ${PROJECT_NAME}/gps_rec.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME gps_rec)
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
