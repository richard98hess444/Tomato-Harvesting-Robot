# Tomato-Harvesting-Robot

## 1. Create a ROS package
Download the repository and make the package with the following
```
mkdit thr_ws
cd thr_ws
git clone https://github.com/richard98hess444/Tomato-Harvesting-Robot.git
catkin_make
```

## 2. Introduction of ```control``` package
The files currently used are 
* data_trans.py
* imu_info.py
* motion.py

These are the files for the motion control of the manipulator.

## 3. Introduction of ```camera``` package
The files currently used are
* center_depth_node.py

These are the files for the tomato depth estimation.

## 4. Introduction of ```yolo_detect``` package
The files currently used are
* detect.py
* detect_usb.py

These are the files for the tomato perception.

## 5. Introduction of ```tmt_robot``` package
The files currently used are
* demo.launch

API from the Moveit.

## 6. Introduction of ```car``` package
The files currently used are
* car_node.py

The vehicle control for the robot.
