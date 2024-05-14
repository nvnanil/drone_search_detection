# drone_search_detection
This is the final project for the course ENAE788M. The project focuses on state estimation and YOLO detction

### Running the code
1. *Clone the package to your ros workspace*
   
2. *Build and source the workspace*
```
cd <path to your ros_ws>/colcon build or colcon build --packages-select coordinate_pkg
source install/setup.bash
```
3. *To run the node to get the location of the object in the camera coordinates, run the following commands in three seperate terminals*
```
voxl-tflite-server
ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node
ros2 run localization_package localization node
```
4. *Subscribe to the /object/pose_camera to get the coordinates of the detected object in the camera frame*
```
ros2 topic echo /object/pose_camera
```
*Messsages are published on the topic whenever the drone detectcs a backpack/handbag in the camera*
