# Tomato-Harvesting-Robot

## ICRA2024 Paper Accepted!
Our works on this tomato harvesting robot has been accepted by ICRA2024! The link of the paper will come as soon as it is available on IEEE explore. Here is a short demo video about the project.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/sg5Gc-BE53Q/0.jpg)](https://www.youtube.com/watch?v=sg5Gc-BE53Q)

## 1. Create a ROS package
Download the repository and make the package with the following
```
mkdit thr_ws
cd thr_ws
git clone https://github.com/richard98hess444/Tomato-Harvesting-Robot.git
catkin_make
```

## 2. Introduction to ```control``` package
The files currently used are 
* data_trans.py
* imu_info.py
* motion.py

These are the files for the motion control of the manipulator.

## 3. Introduction to ```camera``` package
The files currently used are
* center_depth_node.py

These are the files for the tomato depth estimation.

## 4. Introduction to ```yolo_detect``` package
The files currently used are
* detect.py
* detect_usb.py

These are the files for the tomato perception.

## 5. Introduction to ```tmt_robot``` package
The files currently used are
* demo.launch

API from the Moveit.

## 6. Introduction to ```car``` package
The files currently used are
* car_node.py

The vehicle control for the robot.
