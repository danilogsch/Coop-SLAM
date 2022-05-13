# Coop-SLAM

## Data generation on Ignition Gazebo for Segmentation and Tracking CNN's

Tested on Ubuntu 20.04 and ROS Noetic

1. Install this [DART-fork](https://github.com/gazebo-forks/dart) from source:

```
sudo apt-get remove libdart*
sudo apt-get install build-essential cmake pkg-config git
sudo apt-get install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev
sudo apt-get install libopenscenegraph-dev
sudo apt-get install libode-dev

git clone https://github.com/gazebo-forks/dart
cd dart
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

2. Install [Ignition Fortress](https://gazebosim.org/docs/fortress/install_ubuntu_src) from source:

```
sudo apt install python3-pip wget lsb-release gnupg curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
mkdir -p ~/ign_ws/src
cd ~/ign_ws/src
wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-fortress.yaml
vcs import < collection-fortress.yaml
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')

cd ~/ign_ws
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```
Source the workspace in bash:

```
. ~/ign_ws/install/setup.bash
```

3. Clone the fortress_ws and compile ros_ign from source:
```
cd ~
git clone https://github.com/danilogsch/Coop-SLAM
cd /Coop-SLAM/fortress_ws/src
git clone https://github.com/gazebosim/ros_gz/tree/noetic
cd /Coop-SLAM/fortress_ws/
catkin_make
```
Source the workspace in bash:

```
source ~/Coop-SLAM/fortress_ws/devel/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/Coop-SLAM/fortress_ws/src/ign_models"
export IGN_LAUNCH_CONFIG_PATH="$HOME/Coop-SLAM/fortress_ws/src/ign_models/configs"
```

4. Run the following commands to load the depot and Kairos and move the robot with keyboard teleop:
```
ign gazebo empty.sdf
ign launch depot_assets.ign

rosrun ros_ign_bridge parameter_bridge /world/empty/model/rbkairos/link/summit_xl_base_footprint/sensor/summit_xl_front_rgbd_camera_depth_sensor/image@sensor_msgs/Image@ignition.msgs.Image /world/empty/model/rbkairos/link/summit_xl_base_footprint/sensor/summit_xl_front_rgbd_camera_depth_sensor/depth_image@sensor_msgs/Image@ignition.msgs.Image /world/empty/model/rbkairos/link/summit_xl_base_footprint/sensor/summit_xl_front_laser_sensor/scan@sensor_msgs/LaserScan@ignition.msgs.LaserScan /world/empty/model/rbkairos/link/summit_xl_base_footprint/sensor/summit_xl_rear_laser_sensor/scan@sensor_msgs/LaserScan@ignition.msgs.LaserScan /world/empty/model/rbkairos/link/summit_xl_base_footprint/sensor/summit_xl_front_rgbd_camera_depth_sensor/points@sensor_msgs/PointCloud2@ignition.msgs.PointCloudPacked /model/rbkairos/cmd_vel@geometry_msgs/Twist@ignition.msgs.Twist /world/empty/model/rbkairos/link/summit_xl_base_footprint/sensor/imu_sensor/imu@sensor_msgs/Imu@ignition.msgs.IMU

rosrun teleop_twist_keyboard teleop_twist_keyboard.py __name:=teleop1 cmd_vel:=/model/rbkairos/cmd_vel

```



