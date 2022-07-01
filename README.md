# camera_imu_calibration
Camera IMU Calibration using an Extended Kalman Filter

A test dataset can be found [here](https://drive.google.com/file/d/1_81oRKSG5lR9X9jL3swOfnvJNi_wb-Xq/view?usp=sharing)

Make sure to set the correct path to bag file in the launch file.

This repository is our implementation of the Camera-IMU calibration paper [A Kalman filter-based algorithm for IMU-camera calibration by Faraz M Mirzaei, Stergios I Roumeliotis](https://ieeexplore.ieee.org/abstract/document/4637877).

For calculating the Jacobians, I have used the notations and conventions adopted in [OpenVINS](https://github.com/rpng/open_vins). 

## Dependencies
[ROS](http://wiki.ros.org/noetic/Installation/Ubuntu)

This code works on Ubuntu 20.04 and uses the ROS framework for message passing. Standalone installation of OpenCV may not be necessary as this package uses  OpenCV that ships with ROS. 

## Caution
This code base is missing the rotation initialization code that determines I_R_C. If one can provide a decent I_R_C initialization, this will yield results comparable with Kalibr. I'll add the rotation initialization code later. The rotation can be initialized by aligning camera and imu rotation motions. Camera rotation is obtained by tracking a checkerboard target, and imu rotation is obtained by integrating gyroscope measurements. The initialization method is similar to the one implemented [here](https://github.com/unmannedlab/imu_lidar_calibration) for LiDAR-IMU calibration and also presented in the paper [Target-free Extrinsic Calibration of a 3D-Lidar and an IMU](https://arxiv.org/abs/2104.12280) .
