//
// Created by usl on 12/9/20.
//
#ifndef CAMIMUCALIB_PARSE_ROS_H
#define CAMIMUCALIB_PARSE_ROS_H

#include <ros/ros.h>
#include <core/camimucalibManagerOptions.h>

namespace camimucalib_estimator {
    /// This function will load parameters from the ros node handler / parameter server
    camimucalibManagerOptions parse_ros_nodehandler(ros::NodeHandle &nh) {
        /// Our lincalib manager options with defaults
        camimucalibManagerOptions params;

        /// Estimator
        // Main EKF parameters
        ROS_INFO_STREAM("Reading General Estimator Parameters");
        nh.param<bool>("use_fej", params.state_options.do_fej, params.state_options.do_fej);
        nh.param<bool>("use_imuavg", params.state_options.imu_avg, params.state_options.imu_avg);
        nh.param<bool>("use_rk4int", params.state_options.use_rk4_integration, params.state_options.use_rk4_integration);
        nh.param<bool>("calib_camera_timeoffset", params.state_options.do_calib_camera_timeoffset, params.state_options.do_calib_camera_timeoffset);
        nh.param<int>("max_clones", params.state_options.max_clone_size, params.state_options.max_clone_size);
        nh.param<bool>("calib_extrinsics", params.state_options.do_calib_extrinsic, params.state_options.do_calib_extrinsic);
        nh.param<double>("state_init_x_noise", params.state_options.trans_x_noise, params.state_options.trans_x_noise);
        nh.param<double>("state_init_y_noise", params.state_options.trans_y_noise, params.state_options.trans_y_noise);
        nh.param<double>("state_init_z_noise", params.state_options.trans_z_noise, params.state_options.trans_z_noise);
        nh.param<double>("state_init_rx_noise", params.state_options.rot_x_noise, params.state_options.rot_x_noise);
        nh.param<double>("state_init_ry_noise", params.state_options.rot_y_noise, params.state_options.rot_y_noise);
        nh.param<double>("state_init_rz_noise", params.state_options.rot_z_noise, params.state_options.rot_z_noise);
        nh.param<double>("state_init_timeoffset_noise", params.state_options.time_offset_noise, params.state_options.time_offset_noise);

        /// Filter initialization
        ROS_INFO_STREAM("Reading Filter Initialization Parameters");
        nh.param<double>("init_window_time", params.init_window_time, params.init_window_time);
        nh.param<double>("init_imu_thresh", params.init_imu_thresh, params.init_imu_thresh);

        /// Noise
        // Our noise values for inertial sensor
        ROS_INFO_STREAM("Reading IMU Noise Parameters");
        nh.param<double>("gyroscope_noise_density", params.imu_noises.sigma_w, params.imu_noises.sigma_w);
        nh.param<double>("accelerometer_noise_density", params.imu_noises.sigma_a, params.imu_noises.sigma_a);
        nh.param<double>("gyroscope_random_walk", params.imu_noises.sigma_wb, params.imu_noises.sigma_wb);
        nh.param<double>("accelerometer_random_walk", params.imu_noises.sigma_ab, params.imu_noises.sigma_ab);

        // Read update parameters
        ROS_INFO_STREAM("Reading Updater Chi2 Multiplier");
        nh.param<int>("updater_chi2_multiplier", params.updaterOptions.chi2_multiplier, params.updaterOptions.chi2_multiplier);
        nh.param<bool>("updater_do_chi2_check", params.updaterOptions.do_chi2_check, params.updaterOptions.do_chi2_check);

        ROS_INFO_STREAM("Reading Rotation and Noise Update");
        nh.param<double>("updater_rotation_noise", params.updaterOptions.noise_rotation, params.updaterOptions.noise_rotation);
        nh.param<double>("updater_translation_noise", params.updaterOptions.noise_translation, params.updaterOptions.noise_translation);

        /// Global gravity
        ROS_INFO_STREAM("Reading Gravity");
        std::vector<double> gravity = {params.gravity(0), params.gravity(1), params.gravity(2)};
        nh.param<std::vector<double>>("gravity", gravity, gravity);
        assert(gravity.size() == 3);
        params.gravity << gravity.at(0), gravity.at(1), gravity.at(2);

        /// State
        // Timeoffset from camera to IMU
        ROS_INFO_STREAM("Reading initial Timeoffset");
        nh.param<double>("calib_cameraimu_dt", params.calib_cameraimu_dt, params.calib_cameraimu_dt);

        /// Our camera extrinsics transform
        Eigen::Matrix4d I_T_C;
        std::vector<double> matrix_I_T_C;
        std::vector<double> matrix_I_T_C_default = {1,0,0,0,
                                                    0,1,0,0,
                                                    0,0,1,0,
                                                    0,0,0,1};

        /// Camera tracking parameters
        ROS_INFO_STREAM("Reading Camera Tracking Parameters");
        nh.param("checkerboard_dx", params.checkerboard_dx, params.checkerboard_dx);
        nh.param("checkerboard_dy", params.checkerboard_dy, params.checkerboard_dy);
        nh.param("checkerboard_rows", params.checkerboard_rows, params.checkerboard_rows);
        nh.param("checkerboard_cols", params.checkerboard_cols, params.checkerboard_cols);
        nh.param("camera_calibration_file_path", params.camera_calibration_file_path, params.camera_calibration_file_path);

        ROS_INFO_STREAM("Reading camimucalib output file names");
        nh.param("camimu_trajectory_filename", params.camimu_trajectory_filename, params.camimu_trajectory_filename);
        nh.param("camimu_bias_filename", params.camimu_bias_filename, params.camimu_bias_filename);
        nh.param("camimu_velocity_filename", params.camimu_velocity_filename, params.camimu_velocity_filename);
        nh.param("camimu_calib_extrinsic_filename", params.camimu_calib_extrinsic_filename, params.camimu_calib_extrinsic_filename);
        nh.param("camimu_calib_dt_filename", params.camimu_calib_dt_filename, params.camimu_calib_dt_filename);

        ROS_INFO_STREAM("Reading camera pose output trajectory file name");
        nh.param("camerapose_trajectory_filename", params.camerapose_trajectory_filename, params.camerapose_trajectory_filename);

        /// File to read the initial calibration result from
        ROS_INFO_STREAM("Reading the filename to read the initial calibration result to");
        nh.param("init_calibration_result_filename", params.init_calibration_result_filename, params.init_calibration_result_filename);

        std::ifstream initial_calib(params.init_calibration_result_filename);
        std::string word;
        int i = 0; int j = 0;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        while (initial_calib >> word){
            T(i, j) = atof(word.c_str());
            j++;
            if(j>3) {
                j = 0;
                i++;
            }
        }

        // Read in from ROS, and save into our eigen mat
        ROS_INFO_STREAM("Reading initial I_T_C");
        I_T_C = T;
        // Load these into our state
        Eigen::Matrix<double,7,1> camera_imu_extrinsics_ITC;
        camera_imu_extrinsics_ITC.block(0,0,4,1) = rot_2_quat(I_T_C.block(0,0,3,3));
        camera_imu_extrinsics_ITC.block(4,0,3,1) = I_T_C.block(0,3,3,1);
        params.camera_imu_extrinsics = camera_imu_extrinsics_ITC;

        /// File to write the final calibration result to
        ROS_INFO_STREAM("Reading the filename to write the final calibration result to");
        nh.param("calibration_result_filename", params.calibration_result_filename, params.calibration_result_filename);

        return params;
    }
}
#endif //CAMIMUCALIB_PARSE_ROS_H

