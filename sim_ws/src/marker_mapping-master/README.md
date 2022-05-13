# marker_mapping

Node to map visual markers and use them to initialize the robot position (amcl_pose).

### Params
* desired_freq (double)
  * Frequency of the internal control loop. 10 hz by default
* frame_id (string)
  * Frame to refer the saved markers. 'map' by default
* base_frame_id (string)
  * Frame to transform the marker with respect to the base. 'base_link' by default
* publish_saved_markers_tf (bool)
  * Flag to enable the tf publication of the markers
* folder_path (string)
  * System path to save markers config file
* markers_filename (string)
  * Filename of the markers config file.
* load_markers_on_init (bool)
  * Flag to load the saved markers when it starts
* max_marker_id (int)
  * Filter the markers with values higher than this one
* min_marker_id (int)
  * Filter the markers with values lower than this one
 
### Topics

#### Publishers

* /marker_mapping_node/state (marker_mapping/MarkerMappingState)
  * publishes the state of the component
  * the state is READY if a marker is being detected
  * the ids of the detected and saved markers is published
 ```
 state: 
  state: 300
  desired_freq: 10.0
  real_freq: 9.93986320496
  state_description: READY_STATE
 ids_recorded: [0, 1, 2]
 ids_detected: [2]
 ```
* /initialpose (geometry_msgs/PoseWithCovarianceStamped)
  * publishes initial pose (read by amcl)

#### Subscribers

* /ar_pose_marker (ar_track_alvar_msgs/AlvarMarkers)
  * Gets the markers detected by the node ar_trac_alvar
 ```
 header: 
  seq: 10177
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
markers: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1488819030
        nsecs: 565245573
      frame_id: base_link
    id: 2
    confidence: 0
    pose: 
      header: 
        seq: 0
        stamp: 
          secs: 0
          nsecs:         0
        frame_id: ''
      pose: 
        position: 
          x: 0.763555542968
          y: -0.301370897543
          z: 0.69163953281
        orientation: 
          x: -0.479586486245
          y: 0.528171186928
          z: 0.517824549675
          w: -0.472111994409

 ```

### Services

* save_maker (marker_mapping/SaveMarker)
  * Saves the current or desired marker with respect to the frame_id
  * Params:
    * id as [int]. If no id is specified, it saves all the markers received in the topic ar_pose_marker
    * filename as string. It specifies the filename to save the config. If not set, it's saved in the default filename
  * Example:
  ```
  rosservice call /marker_mapping_node/save_maker "id: []
  filename: ''"
  ```
* init_pose_from_marker (marker_mapping/InitPoseFromMarker)
  * If a markers is being detected and it has been previously saved with respect to frame_id, it calculates the robot position and sends the initpose to the amcl node.
  * Params:
    * id as [int]. The id of the marker to use for the initialization. If no id provided, the node will use the first one received in ar_pose_marker as long as it has been mapped previously. 
  * Example:
 ```
 rosservice call /marker_mapping_node/init_pose_from_marker "id: []"
 ```
 
