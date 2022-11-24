# Coop-SLAM

## Data generation on Ignition Gazebo (Garden) for Segmentation and Object Tracking CNN's

Tested on Ubuntu Jammy (22.04) and ROS2 Humble

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

2. Install [Gazebo Garden](https://gazebosim.org/docs/garden/install_ubuntu_src) from source:

```
sudo apt install python3-pip wget lsb-release gnupg curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
mkdir -p ~/gz_garden_ws/src
cd ~/gz_garden_ws/src
wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-fortress.yaml
vcs import < collection-fortress.yaml
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')
```
Observation: In case mecanum wheels robots are used, gz-sim and gz-math need to be replaced for these ones ([gz-sim](https://github.com/danilogsch/gz-sim/tree/gz-sim7-mwodom) and [gz-math](https://github.com/danilogsch/gz-math/tree/gz-math7-mwodom)) in order to be able to publish odometry and tf topics (needed to navigate the robot). An alternative would be to use their odometry [plugin publisher](https://github.com/gazebosim/gz-sim/tree/gz-sim7/src/systems/odometry_publisher).
```
cd ~/gz_garden_ws
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```
Source the workspace in bash:

```
. ~/gz_garden_ws/install/setup.bash
```
3. Install [nav2](https://navigation.ros.org/build_instructions/index.html#build-nav2) from source and [moveit](https://moveit.ros.org/install-moveit2/binary/). Add the following to your bash to use Cyclone DDS:
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
4. Clone the humble_ws and compile ros_gz from source:
```
cd ~
git clone https://github.com/danilogsch/Coop-SLAM
cd /Coop-SLAM/humble_ws/src
git clone https://github.com/gazebosim/ros_gz/tree/humble
cd /Coop-SLAM/humble_ws/
export GZ_VERSION=garden
colcon build
```
Source the workspace in bash:

```
source ~/Coop-SLAM/humble_ws/install/local_setup.bash
```
5. Change the datagen_scripts/gz_configs/launch_params.yaml according to the desired behaviour. Important: Modify the paths in the first 3 lines to your absolute paths.
Run the following commands in different terminals to listen to the image data topics (this is needed to save data properly) and run the launch script 5 times in a row:
```
gz topic -e -t /semantic/labels_map
gz topic -e -t /boxes_full_2d_image
gz topic -e -t depth_camera
gz topic -e -t /boxes_3d_image
gz topic -e -t /boxes_visible_2d_image
gz topic -e -t /semantic/colored_map
gz topic -e -t /panoptic/colored_map

for counter in {1..5}; do ros2 launch datagen_scripts random_spawner_launch.py; done

```
Segmentation example output:

![](https://github.com/danilogsch/Coop-SLAM/blob/main/rgb.gif)
![](https://github.com/danilogsch/Coop-SLAM/blob/main/segmentation.gif)

Documentation about available models, folders structure, parameters and how to add new models will be added soon!



