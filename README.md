# LeRobot

ROS 2 Jazzy robotics learning project.

## Completed Milestones

### Milestone 1

* ROS 2 Jazzy installation
* Workspace creation
* SO101 robot description package
* Robot visualization in RViz

### Milestone 2

* USB camera integration
* ROS image topic subscription
* OpenCV image processing
* YOLOv8 real-time object detection

Objects currently detected:

* Person
* Cell phone
* Bottle
* Chair
* Laptop
* Other YOLOv8 supported classes

## Workspace Structure

```text
ros2_ws/
├── src/
│   ├── camera_vision/
│   └── so101_description/
```

## Requirements

* Ubuntu 24.04
* ROS 2 Jazzy
* Python 3.12
* OpenCV 4.8.1
* Ultralytics YOLOv8

## Running Camera Detection

Terminal 1:

```bash
ros2 run usb_cam usb_cam_node_exe
```

Terminal 2:

```bash
source ~/ai_projects/yolo_env/bin/activate
python ~/ros2_ws/src/camera_vision/camera_vision/camera_viewer.py
```

## Current Status

Real-time object detection working successfully.

Next goal:

* Publish detections as ROS topics
* Build robot perception pipeline
* Integrate vision with robot behavior

