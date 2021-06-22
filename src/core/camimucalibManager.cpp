//
// Created by usl on 6/21/21.
//

#include "camimucalibManager.h"

using namespace camimucalib_type;
using namespace camimucalib_estimator;

camimucalib_estimator::camimucalibManager::camimucalibManager(camimucalib_estimator::camimucalibManagerOptions &param_) {
    /// Startup  message
    // Nice startup message
    printf("=======================================\n");
    printf("Camera Inertial Calibration ON-MANIFOLD EKF IS STARTING\n");
    printf("=======================================\n");

    /// Nice debug
    this->params = param_;
    params.print_estimator();
    params.print_noise();
    params.print_state();
    params.print_trackers();

    /// File to store cameraimu trajectory
    trajfile_csv.open(params.camimu_trajectory_filename);
    bias_csv.open(params.camimu_bias_filename);
    velocity_csv.open(params.camimu_velocity_filename);
    calib_extrinsic_csv.open(params.camimu_calib_extrinsic_filename);
    calib_dt_csv.open(params.camimu_calib_dt_filename);

    /// Create the state
    state = new State(params.state_options);

    /// Time offset from Camera to IMU
    Eigen::VectorXd temp_cameraimu_dt;
    temp_cameraimu_dt.resize(1);
    temp_cameraimu_dt(0) = params.calib_camimu_dt;
    state->_calib_dt_CAMERAtoIMU->set_value(temp_cameraimu_dt);
    state->_calib_dt_CAMERAtoIMU->set_fe(temp_cameraimu_dt);

    /// Extrinsic Calibration
    state->_calib_CAMERAtoIMU->set_value(params.cam_imu_extrinsics);
    state->_calib_CAMERAtoIMU->set_fe(params.cam_imu_extrinsics);

    /// Propagator
    propagator = new Propagator(params.imu_noises, params.gravity);

    /// Initializer
    initializer = new InertialInitializer(params.gravity, params.init_window_time, params.init_imu_thresh);

    /// Make the Updater
    updaterCameraTracking = new UpdaterCameraTracking(params.updaterOptions);

    /// Initialize the Camera Pose Object
//    cameraPoseTracker = std::make_shared<camimucalib_core::cameraTracking>(params.checkerboard_dx, params.checkerboard_dy,
//                                                                           params.checkerboard_rows, params.checkerboard_cols,
//                                                                           params.camera_calibration_file_path);
}

bool camimucalib_estimator::camimucalibManager::try_to_initialize() {
    /// Returns from our initializer
    double time0;
    Eigen::Matrix<double, 4, 1> q_GtoI0;
    Eigen::Matrix<double, 3, 1> b_w0, v_I0inG, b_a0, p_I0inG;

    /// Try to initialize the system
    /// We will wait for a jerk
    bool wait_for_jerk = true;
    bool success = initializer->initialize_with_imu(time0, q_GtoI0, b_w0,v_I0inG,
                                                    b_a0, p_I0inG, wait_for_jerk);

    /// Return if it failed
    if(!success) {
        return false;
    }

    /// Make big vector (q, p, v, bg, ba)
    Eigen::Matrix<double,16,1> imu_val;
    imu_val.block(0,0,4,1) = q_GtoI0;
    imu_val.block(4,0,3,1) << 0,0,0;
    imu_val.block(7,0,3,1) = v_I0inG;
    imu_val.block(10,0,3,1) = b_w0;
    imu_val.block(13,0,3,1) = b_a0;
    state->_imu->set_value(imu_val);
    state->_imu->set_fe(imu_val);
    state->_timestamp = time0;
    startup_time = time0;

    printState();
    return true;
}

void camimucalib_estimator::camimucalibManager::feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am) {
    /// Push into the propagator
    propagator->feed_imu(timestamp, wm, am);

    /// Push into our initializer
    if(!is_initialized_camimucalib) {
        initializer->feed_imu(timestamp, wm, am);
    }
}

void camimucalib_estimator::camimucalibManager::feed_measurement_camera(double timestamp, cv::Mat image_in) {
    if(!is_initialized_camimucalib) {
        is_initialized_camimucalib = try_to_initialize();
        if(!is_initialized_camimucalib)
            return;
    }

//    cameraPoseTracker->feedImage(timestamp, image_in);
//
//    do_propagate_update(timestamp);
//
//    if(state->_clones_IMU.size() == 1) {
//        /// G_T_I1
//        Eigen::Matrix<double, 4, 1> q_GtoI1 = state->_imu->quat();
//        Eigen::Matrix3d I1_R_G = camimucalib_core::quat_2_Rot(q_GtoI1);
//        Eigen::Matrix3d G_R_I1 = I1_R_G.transpose();
//        Eigen::Vector3d G_t_I1 = state->_imu->pos();
//        G_T_I1.block(0, 0, 3, 3) = G_R_I1;
//        G_T_I1.block(0, 3, 3, 1) = G_t_I1;
//    }
//    /// Printing for debug
//    printState();
}

void camimucalib_estimator::camimucalibManager::do_propagate_update(double timestamp) {
    if(state->_timestamp >= timestamp) {
        printf(YELLOW "Stepping back in time!!! (prop dt = %3f)\n" RESET, (timestamp-state->_timestamp));
        return;
    }
}