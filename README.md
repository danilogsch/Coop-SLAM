# Coop-SLAM

##Multi-source Multi-object detection and segmentation from 3D LiDAR BEVs. 

Available modes:
- Binnary segmentation (what is actually published on topic)
- Instance segmentation (different colors for each instance)
- Semantic segmentation (different colors for each class)

For generating more data with the available models, check out the [datagen readme](https://github.com/danilogsch/Coop-SLAM/blob/main/DataGen_README.md). 

Scripts for transforming the generated data into kitti-like dataset format are available on the jupyter notebook: Data_transfrom.ipynb.
For trainning/testing/validating the model use the jupyter notebook: resnet50_fpnkeypoint_fcn.ipynb

If you want to use my generated synth_depot_dataset for trainning/testing/validating the model, download it from this [google drive folder](https://drive.google.com/drive/folders/1HEUbte6S7996iOndTYddwkNz7qlJFa4Y?usp=sharing) and put the contents under humble_ws/synth_depot_dataset.

The model weights and utils obtained during trainning will be available after peer-review at this link. Put it on checkpoints/fpn_resnet_50_semseg.

Tested on Ubuntu Jammy (22.04) and ROS2 Humble with Ignition Gazebo (Garden). Future release will use Harmonic LTS with DockerFile to ease installion and usage.

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
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
mkdir -p ~/gz_garden_ws/src
cd ~/gz_garden_ws/src
wget https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-garden.yaml
vcs import < collection-garden.yaml
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
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
Source the workspace in bash (include lines in ~/.bashrc), change path if needed:

```
source ~/Coop-SLAM/humble_ws/install/local_setup.bash
export COOP_SLAM_PATH=~/Coop-SLAM
```

To install all python packages/dependencies
```
pip install -r requirements.txt
```

For testing and evaluation, the simulation run once for each robot using the network and saves its output in a bag file. Example for robot "rbkairos_migration_sensor_config_3_1" in "ign_models/coop_slam.sdf":
Since it has an simulated person, change the absolute path at line 170 of the coop_slam.sdf file according to your "humble_ws/src/coop_detection_and_mapping/human_data/panoptic" absolute path.
```
gz sim coop_slam.sdf

#On different terminal (since the world contains a person):
ros2 run coop_detection_and_mapping human_data_saver --ros-args -p use_sim_time:=True

# On a diferent terminal (bridge simulation and ROS2):
ros2 run ros_gz_bridge parameter_bridge '/human_vehicle/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image' '/human_vehicle/panoptic/colored_map@sensor_msgs/msg/Image[gz.msgs.Image' '/rbkairos_migration_sensor_config_3_1/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock' '/model/rbkairos_migration_sensor_config_3_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' '/model/rbkairos_migration_sensor_config_3_1/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' '/model/rbkairos_migration_sensor_config_3_2/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' '/model/human_vehicle/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' '/model/youbot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' '/model/pallet_box_mobile/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' '/model/aws_robomaker_warehouse_PalletJackB_01/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' '/model/cart_model2/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' --ros-args -r '/model/rbkairos_migration_sensor_config_3_0/pose:=/tf' -r '/model/rbkairos_migration_sensor_config_3_1/pose:=/tf' -r '/model/rbkairos_migration_sensor_config_3_2/pose:=/tf' -r '/model/human_vehicle/pose:=/tf' -r '/model/youbot/pose:=/tf' -r '/model/pallet_box_mobile/pose:=/tf' -r '/model/aws_robomaker_warehouse_PalletJackB_01/pose:=/tf' -r '/model/cart_model2/pose:=/tf' 

# On a diferent terminal (to give the models velocity comands, press play on simulation after these commands):
gz topic -t "/rbkairos_migration_sensor_config_3_0/cmd_vel" -m gz.msgs.Twist -p "linear: {y: -0.5}, angular: {z: 0}";
gz topic -t "/rbkairos_migration_sensor_config_3_1/cmd_vel" -m gz.msgs.Twist -p "linear: {y: -0.45}, angular: {z: 0}";
gz topic -t "/rbkairos_migration_sensor_config_3_2/cmd_vel" -m gz.msgs.Twist -p "linear: {y: -0.4}, angular: {z: 0}";
gz topic -t "/human_vehicle/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.9}, angular: {z: 0}";
gz topic -t "/youbot/cmd_vel" -m gz.msgs.Twist -p "linear: {y: 0.4}, angular: {z: -0.2}"

# On a diferent terminal (actual detection node, change hard-coded topic name to work with other robots):
ros2 run coop_detection_and_mapping detection_publisher --ros-args -p use_sim_time:=True

# On a diferent terminal (to record the detections):
ros2 bag record tf seg_out_topic detections_topic -o bag2 --use-sim-time
```

Merged bags with multiple detections can be found in humble_ws/coop_bags folder.
To run the multi-robot SLAM node:

```
ros2 bag play humble_ws/coop_bags/merged_bag
ros2 run coop_detection_and_mapping centralized_ogm_builder --ros-args -p use_sim_time:=True
```

Resulting maps can be visualized in cv2 and are published in nav2_msgs/msg/OccupancyGrid msg type under "map" topic. Code to save the dictionary with poses and map images is commented on the code. Available parameters:

- empty_map_path: Path to the empty resized map of the environment.
- min_conf: Minimum confidence value to consider detection. (default 0.3)
- symmetric_classes: List of class names that have a symmetric shape for yaw rotation correction.
- is_ogm: Perform instance segmentation and overrides ss_ogm. (default False)
- ss_ogm: Perform semantic segmentation. (default True)
- num_classes: total number of classes. (default 14)
- max_vyaw: Maximum angular velocity for yaw rotation correction. (default 2)
- min_pixels_per_occ: Minimum percentage of pixels in the bounding box to consider detection. (default 0.05)[5%]
- x_bound: Map limits in x axis meters. (default [-14.0,14.0])
- y_bound: Map limits in x axis meters. (default [-7.0,7.0])
- sensor_offset_x: Distance from sensor to robots base_link in pixels. (default of our rbkairos configuration 21.7968)
- visualize: Wether to visualize map with cv2.imshow or not. (default True)
- publish_map: Wether to publish nav2_msgs/msg/OccupancyGrid msg type under "map" topic or not. (default True)

Semantic Segmentation example:

![](https://github.com/danilogsch/Coop-SLAM/blob/main/coopslam.gif)

Paper results and videos available on [this link](https://drive.google.com/drive/folders/1hF9q2466QbS1hW0QMyjwjuKk0YqQm2Uo?usp=sharing).

To generate ground truth:

```
# If using any actor with human_vehicle, while simulation runs:
ros2 run coop_detection_and_mapping human_data_saver --ros-args -p use_sim_time:=True

# to generate ground truth:
ros2 run coop_detection_and_mapping groundtruth_gen --ros-args -p use_sim_time:=True
ros2 bag play humble_ws/coop_bags/bag1 #Or any bag with tfs recorded

# to run evaluation script and plot errors:
python3 coop_evaluation.py
```


## Folder Structure (most important files)

```
├── humble_ws/src
│   ├── arm_gz_control (Package for joint control of robotic arms in Gazebo)
│   ├── ign_models (Folder containing available models meshes and URDFs)
│   ├── coop_detection_and_mapping
│   │   ├── detection_publisher (Detection node listens to LiDAR and publishes seg_out_topic and detections_topic)
│   │   └── centralized_ogm_builder (Centralized object tracking and map builder node)
│   ├── detection_messages (Unused custom messages for detections to be implemented)
│   └── datagen_scripts (Main data generation package)
│       ├── datagen_scripts (Python scripts)
│       ├── gz_configs (Configuration files defining the behaviour of the data generation)
│       │   ├── data_frame_dict.csv (Table with available models and their parameters)
│       │   └── launch_params.yaml (Launch parameters that configure the data generation algorithm)
│       ├── moveit_configs (MoveIt related configuration files)
│       ├── nav2_configs (Navigation2 related configuration files)
│       ├── jupyter_notebooks (Jupyter notebooks for data conversion and CNN training (detectron2))
│       └── launch
│           └── random_spawner_launch.py (ROS2 python launcher for randomly generation of data)
├── humble_ws/data_dump (Default folder to store generated data)
├── humble_ws/instances_footprint (Footprints of each rigid model)
├── humble_ws/synth_depot_dataset (generated dataset folder)
├── humble_ws/r2b_dataset (manipulation of NVIDIA's r2b_dataset)
├── humble_ws/resnet50_fpnkeypoint_fcn.ipynb (Jupyter Notebook to define and train/test/val model)
├── humble_ws/Data_transfrom.ipynb (Auxiliary Jupyter Notebook to transform data from data_dump into synth_depot_dataset)
├── README.md
├── Datagen_README
└── Adding_new_models.md

```
## Additional Info

Two sequences of NVIDIA's r2b_dataset were used to evaluate the model: r2b_hallway and r2b_storage. Pure python scripts in r2b_dataset folder were used to extract LiDAR/RGB data, generate BEV images and a custom annotation tool was used to annotate the segmentation and oriented bounding boxes of humans in these BEV images to serve as groud_truth. Original dataset can be found [here](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/r2bdataset2023) under [Creative Commons - Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/) license, to reproduce the extraction and annotation download the sequences and put the rosbags into humble_ws/r2b_dataset/r2b_hallway and humble_ws/r2b_dataset/r2b_storage respectively. 

Example for extracting data from one rosbag sequence:

```
cd humble_ws/r2b_dataset
python3 extract_lidar.py
python3 extract_rgb.py
#To move syncronized images with LiDAR:
python3 move_images.py
#To annotate oriented bbox:
python3 annotation.py
#To annotate segmentation:
python3 seg_annotation.py

```



