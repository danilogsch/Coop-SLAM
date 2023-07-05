# Adding new models to generate data (Robots, Actors and Objects)

Obs: If you have description files for a model in the Classic Gazebo simulator, This [migration guide](https://github.com/osrf/subt/wiki/Model-Migration-Guide) is a good starting point to migrate it to the new Gazebo Sim.

## Adding a robot

Basic folder structure for the model inside /ign_models:

```
── humble_ws/src/ign_models/{$name}
   ├── /meshes (.dae/.stl files)
   ├── /material (texture files)
   ├── /urdf
   │     ├── {$name}.urdf
   └── model.sdf

```

For navigation, each model uses a simulated Lidar with 360 degrees of view, placed close to the ground. The generic frame links and joints on the .sdf and .urdf files should match the following examples according to the robots driving system:

1. For diferential-driven robots (robot_type: 0 in data_frame_dict.csv):

```
{$name}/base_footprint
   ├── base_left_wheel_joint ── {$name}_left_wheel
   ├── base_right_wheel_joint ── {$name}_right_wheel
   └── {$name}/front_laser_link

```
      
2. For Mecanum-wheeled-driven robots (robot_type: 1 in data_frame_dict.csv):

```
{$name}/base_footprint
   ├── front_right_wheel_joint ── front_right_wheel
   ├── front_left_wheel_joint ── front_left_wheel
   ├── rear_right_wheel_joint ── rear_right_wheel
   ├── rear_left_wheel_joint ── rear_left_wheel
   └── {$name}/front_laser_link

```

3. For 6 DoF robotic arms (has_arm: 1 in data_frame_dict.csv):
Based on the UR-10 robotic arm, joint names: "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",                   "wrist_1_joint","wrist_2_joint","wrist_3_joint"

4. For 5 DoF robotic arms (has_arm: 2 in data_frame_dict.csv):
Based on the youbot robotic arm, joint names: "arm_joint_1","arm_joint_2","arm_joint_3",                            "arm_joint_4","arm_joint_5"


Moveit2 related description and configuration files are stored in src/datagen_scripts/moveit_configs folder
The Gazebo plugins used for Joint Position, Joint control, Odometry and Navigation can be mirroed from the available models. Make sure to attach them to the correct frames/joints and use the same topic name format as the examples.


## Adding an actor

Basic folder structure for the model inside /ign_models:

```
── humble_ws/src/ign_models/{$name}
   ├── /meshes (.dae animation and skeleton files)
   └── model.sdf

```

In order to control the actors, the plugin "follow-actor-system" was used to follow an invisible differential robot, this robot does not have <visual> tags, meaning that it will not be rendered into simulation nor detected by any sensor. It can be seen in the file "src/datagen_scripts/gz_configs/human_vehicle_model/actor_wrd_tmpl.sdf" but there is no need to modify it.

## Adding an object

Basic folder structure for the model inside /ign_models:

```
── humble_ws/src/ign_models/{$name}
   ├── /meshes (.dae/.stl files)
   ├── /material (texture files)
   └── model.sdf

```

## Camera settings:
Camera parameters like width, height, HFOV, gaussian noise can be configured in the file "gz_configs/camera_tmpl.sdf" under each specific camera tag.

## Environment model:
There is only one environment model, but one could change it in the end of the file "gz_configs/human_vehicle_model/wrd_bgn_tmpl.sdf". In this case, use any robot in this environment and a SLAM method to generate an Occupancy Grid Map, them save it in the datagen_scripts/map folder replacing the existing one.
