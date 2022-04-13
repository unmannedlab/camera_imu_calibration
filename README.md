# camera_imu_calibration
Camera IMU Calibration using an Extended Kalman Filter

A test dataset can be found [here](https://drive.google.com/file/d/1_81oRKSG5lR9X9jL3swOfnvJNi_wb-Xq/view?usp=sharing)

Make sure to set the correct path to bag file in the launch file.

This repository is our implementation of the Camera-IMU calibration paper [A Kalman filter-based algorithm for IMU-camera calibration by Faraz M Mirzaei, Stergios I Roumeliotis](https://ieeexplore.ieee.org/abstract/document/4637877).

For calculating the Jacobians, I have used the notations and conventions adopted in [OpenVINS](https://github.com/rpng/open_vins). 

## Dependencies
[ROS](http://wiki.ros.org/noetic/Installation/Ubuntu)

This code works on Ubuntu 20.04 and uses the ROS framework for message passing. Standalone installation of OpenCV may not be necessary as this package uses  OpenCV that ships with ROS. 
