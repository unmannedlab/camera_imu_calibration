//
// Created by usl on 12/8/20.
//

#ifndef CAMIMUCALIB_ESTIMATOR_CAMIMUCALIBMANAGEROPTIONS_H
#define CAMIMUCALIB_ESTIMATOR_CAMIMUCALIBMANAGEROPTIONS_H

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include <state/StateOptions.h>
#include <state/Propagator.h>
#include <update/UpdaterOptions.h>
#include <utils/color.h>
#include <utils/quat_ops.h>


using namespace std;
using namespace camimucalib_core;

namespace camimucalib_estimator {
    /// Struct which stores all options needed for state estimation
    struct camimucalibManagerOptions {
        /// ESTIMATOR ===============================

        StateOptions state_options;

        // Amount of time we will initialize over (seconds)
        double init_window_time = 1.0;

        //  Variance threshold on our acceleration to be classified as moving
        double init_imu_thresh = 1.0;

        // This function will print out all estimator settings loaded.
        void print_estimator() {
            printf("ESTIMATOR PARAMETERS:\n");
            state_options.print();
            printf("\t- init_window_time: %.2f\n", init_window_time);
            printf("\t- init_imu_thresh: %.2f\n", init_imu_thresh);
        }

        /// NOISE / CHI2 ============================
        // IMU noise (gyroscope and accelerometer)
        Propagator::NoiseManager imu_noises;

        // Update options for estimator (noise and chi2 multiplier)
        UpdaterOptions updaterOptions;

        // This function will print out all noise parameters loaded.
        void print_noise() {
            printf("NOISE PARAMETERS:\n");
            imu_noises.print();
            printf("\tUpdater Estimator Feats:\n");
            updaterOptions.print();
        }

        /// STATE DEFAULTS ============================
        // Gravity in global frame
        Eigen::Vector3d gravity = {0.0, 0.0, 9.81};

        // Time offset between camera and IMU
        double calib_cameraimu_dt = 0.0;

        //  IMU extrinsics (q_LtoI, p_LinI). Note the
        // difference between "to" and "in"
        Eigen::Matrix<double, 7, 1> camera_imu_extrinsics;

        // This function will print out all state defaults loaded.
        void print_state() {
            printf("STATE PARAMETERS:\n");
            printf("\t- gravity: %.3f, %.3f, %.3f\n", gravity(0), gravity(1), gravity(2));
            printf("\t- calib_camimu_dt: %.4f\n", calib_cameraimu_dt);
            std::cout << "cam_imu_extrinsic(0:3):" << endl << camera_imu_extrinsics.block(0,0,4,1).transpose() << std::endl;
            std::cout << "cam_imu_extrinsic(4:6):" << endl << camera_imu_extrinsics.block(4,0,3,1).transpose() << std::endl;
        }

        /// Camera Pose Tracking
        double checkerboard_dx = 0.05; // 5 cm
        double checkerboard_dy = 0.05; // 5 cm
        int checkerboard_rows = 6;
        int checkerboard_cols = 7;
        std::string camera_calibration_file_path = "/home/usl/catkin_ws/src/camera_imu_calibration/config/pylon_calib.yaml";
        void print_trackers() {
            printf("CAMERA Tracking PARAMETERS:\n");
            printf("checkerboard_dx: %0.4f\n", checkerboard_dx);
            printf("checkerboard_dy: %0.4f\n", checkerboard_dy);
            printf("checkerboard_rows: %d\n", checkerboard_rows);
            printf("checkerboard_cols: %d\n", checkerboard_cols);
            printf("camera_calibration_file_path: %s\n", camera_calibration_file_path.c_str());
        }

        /// CSV filename as output
        std::string camimu_trajectory_filename;
        std::string camimu_bias_filename;
        std::string camimu_velocity_filename;
        std::string camimu_calib_extrinsic_filename;
        std::string camimu_calib_dt_filename;
        std::string camerapose_trajectory_filename;
        std::string reprojection_error_filename;

        /// Initial calibration result filename
        std::string init_calibration_result_filename;

        /// Final calibration result filename
        std::string calibration_result_filename;
    };
};
#endif //CAMIMUCALIB_ESTIMATOR_CAMIMUCALIBMANAGEROPTIONS_H
