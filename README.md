# drone_search_detection_localization
This is the final project for the course ENAE788M. The project focuses on state estimation and YOLO detction

### Running the code
1. *Clone the package to your ros workspace*
   
2. *Build and source the workspace*
```
cd <path to your ros_ws>/colcon build or colcon build --packages-select coordinate_pkg
source install/setup.bash
```
3. *To run the node to start the detection*
```
voxl-tflite-server
ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node
ros2 run localization_package inertial_camera_coord.py
ros2 run localization_package localization_inertial_node
ros2 run px4_ros_com search_for_tag
```
4. *Subscribe to the /object/pose_inertial to get the coordinates of the detected object in the camera frame*
```
ros2 topic echo /object/pose_inertial
```
*Messsages are published on the topic whenever the drone detectcs a backpack/handbag in the camera*
