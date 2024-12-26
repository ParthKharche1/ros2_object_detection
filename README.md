# ros2_object_detection
This package provides a framework for evaluating and comparing different object detection algorithms within a ROS 2 environment. Currently, it includes YOLOv10. Support for additional algorithms will be added in future updates.

Key Features:
1. Modular Design: Easily integrate new object detection models.
2. ROS 2 Integration: Seamlessly publish and subscribe to ROS 2 topics.

git clone -> https://github.com/ParthKharche1/ros2_object_detection.git
              cd ros2_object_detection

# STEPS:
## 1. Create the ROS2 Package
Navigate to your ROS2 workspace and create a new package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python yolov10_detection
```

  Add dependencies to the package.xml
  ```bash
   <exec_depend>rclpy</exec_depend>
   <exec_depend>sensor_msgs</exec_depend>
   <exec_depend>cv_bridge</exec_depend>
   <exec_depend>numpy</exec_depend>
```
Update the setup.py
```bash
    entry_points={
    'console_scripts': [
          'yolo_detection = yolov10_detection.yolo_detection:main',
       ],
  },
```

## 2. Write the Detection Node (yolo_detection.py)
[CODE](https://github.com/ParthKharche1/ros2_object_detection/blob/main/yolov10_detection.py)

## 3. Build the package:
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
## 4. Run the YOLO detection node:
    ros2 run yolov10_detection yolo_detection

### ROS2 Topics Published
```bash 
/yolo_detections (std_msgs/String)
```
 Contains detection information like class names and confidence scores.
```bash
/yolo_frames (sensor_msgs/Image)
```
Annotated frames as ROS2 image messages.

You can visualize the frames using a tool like rqt_image_view:
```bash
ros2 run rqt_image_view rqt_image_view
```
This setup integrates your YOLO detection pipeline with ROS2, making it useful for robotic applications that require object detection.

## License

[MIT](https://choosealicense.com/licenses/mit/)






