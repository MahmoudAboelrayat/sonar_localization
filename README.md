# sonar_localization

A ROS 2 package for underwater sonar-based localization using point cloud registration (GICP/VGICP), loop closure, and prior map localization.

## Dependencies

### 1. ROS 2

Install [ROS 2](https://docs.ros.org/en/humble/Installation.html) (Humble or later).

### 2. System Packages

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-nav-msgs \
  ros-$ROS_DISTRO-geometry-msgs \
  ros-$ROS_DISTRO-visualization-msgs \
  libeigen3-dev \
  libpcl-dev
```

### 3. GTSAM

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc) && sudo make install
```

### 4. fast_gicp

```bash
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
cd fast_gicp && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

### 5. small_gicp

```bash
git clone https://github.com/koide3/small_gicp.git
cd small_gicp && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

## Installation

```bash
mkdir -p ~/thesis_ws/src && cd ~/thesis_ws/src
git clone git@github.com:MahmoudAboelrayat/sonar_localization.git
cd ~/thesis_ws
colcon build --packages-select sonar_localization
source install/setup.bash
```

## Package Structure

```
sonar_localization/
├── src/                  # C++ nodes
│   ├── gicp_odom.cpp
│   ├── fastgicp_odom.cpp
│   ├── fastvgicp_odom.cpp
│   ├── mapping.cpp
│   ├── loopclosure_vgicp.cpp
│   ├── loopclosure_samll_vgicp.cpp
│   ├── prior_map_localizer.cpp
│   ├── prior_map_publisher.cpp
│   └── imu_preintegration.cpp
├── sonar_localization/   # Python nodes
│   ├── dvl_bridge.py
│   ├── imu_bridge.py
│   ├── depth_bridge.py
│   ├── odom_tf.py
│   ├── pointcloud_features.py
│   ├── pointcloud_intensity.py
│   ├── sonar_noise.py
│   ├── pub_icp_data.py
│   ├── ekf_to_csv_logger.py
│   └── gt_map.py
├── launch/               # Launch files
├── config/               # Parameter files
└── rviz/                 # RViz configurations
```
