# SEARCH, DETECTION, AND LOCALIZATION OF AN OBJECT USING AERIAL DRONE
This is the final project for the course ENAE788M: Hands on Autonomous Aerial Robotics. The project focuses on state estimation, YOLO detection, and localization of an object using a tracking camera

### Running the code
1. *Clone the package to your ros workspace*
   
2. *Build and source the workspace*
```
cd <path_to_ros_ws>
colcon buiild
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
