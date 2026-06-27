# LeRobot

ROS 2 Jazzy robotics learning project.

## Completed Milestones

### Milestone 1: SO101 Robot Visualization

* ROS 2 Jazzy installation
* Workspace creation
* SO101 robot description package
* Robot visualization in RViz

### Milestone 2: Camera + YOLO Object Detection

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

### Milestone 3: SO101 Gazebo Pick-and-Place with Wrist Camera

* SO101 arm spawned in Gazebo
* ROS 2 controllers active
* Gripper controller working
* 2 cm cube spawned and stable
* Virtual cube attach/release system working
* Automatic pick-place sequence working
* One-command full pick-place demo working
* Wrist camera mounted near gripper
* Gazebo wrist camera topic publishing
* ROS 2 camera bridge working
* Wrist camera image visible in `rqt_image_view`
* Full pick-place demo works while wrist camera streams

## Workspace Structure

```text
ros2_ws/
├── src/
│   ├── camera_vision/
│   ├── so101_description/
│   ├── so101_gazebo/
│   └── so101_moveit_config/
```

## Requirements

* Ubuntu 24.04
* ROS 2 Jazzy
* Gazebo Sim
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

## Running SO101 Gazebo Pick-and-Place with Wrist Camera

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_gazebo full_auto_pick_place_with_camera.launch.py
```

Terminal 2, to view wrist camera:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run rqt_image_view rqt_image_view /wrist_camera/image
```

## Important ROS Topics

```text
/wrist_camera/image
/wrist_camera/camera_info
/pick_place_state
/joint_states
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

## Current Working Safe Poses

```text
pick_pose  = [0.0, -0.5, 0.5, -0.3, 0.0]
place_pose = [0.6, -0.5, 0.5, -0.3, 0.0]
home       = [0.0, 0.0, 0.0, 0.0, 0.0]
```

## Current Status

Working:

* Real-time YOLO camera detection
* SO101 Gazebo automatic pick-place
* Wrist camera image streaming from Gazebo to ROS 2

Next goal:

* Detect cube from wrist camera image
* Publish detected object center as a ROS 2 topic
* Later convert image pixel + depth into 3D robot coordinates


---

## Final Gazebo V1 Demo: Camera-Centered Pick and Place

This launch runs the complete Gazebo simulation pipeline:

- Spawn SO101 robot
- Start controllers
- Start wrist camera bridge
- Move cube into wrist camera view
- Detect red cube using OpenCV color detection
- Compute visual servo error
- Center cube in camera view using shoulder pan and wrist flex
- Trigger automatic pick-place after centering
- Attach cube virtually
- Move cube to place pose
- Release cube
- Finish sequence

Run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_gazebo vision_center_then_pick.launch.py
```


```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_gazebo final_so101_gazebo_pick_place.launch.py
```

## V3: MoveIt + Wrist Camera + Gazebo Pick-Place

### Final command

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_moveit_cpp v3_final_moveit_pick_place.launch.py
```

---

## V4: Camera-Derived Cube Pose Pick-Place

### Final command

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_moveit_cpp v4_camera_derived_pick_place.launch.py
```
Gazebo SO101 robot
        ↓
wrist camera image
        ↓
red cube detector
        ↓
/camera_cube_error
        ↓
V4 camera-derived cube pose publisher
        ↓
/pick_cube_pose
        ↓
MoveIt shoulder pan refinement
        ↓
grasp, attach, lift, place, release

V4 CAMERA-DERIVED /pick_cube_pose
CAMERA ALIGNMENT PASSED
V4 camera-derived pose after alignment
V4 refining shoulder_pan
PAN_REFINE_CAMERA_DERIVED
PRE_GRASP_CAMERA_DERIVED
CUBE ATTACHED
CUBE LIFTED
CUBE RELEASED TO GROUND
V4 MOVEIT CAMERA-DERIVED PICK COMPLETE


