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
    visodom_csv.open(params.camerapose_trajectory_filename);
    repErr_csv.open(params.reprojection_error_filename);

    /// Create the state
    state = new State(params.state_options);

    /// Time offset from Camera to IMU
    Eigen::VectorXd temp_cameraimu_dt;
    temp_cameraimu_dt.resize(1);
    temp_cameraimu_dt(0) = params.calib_cameraimu_dt;
    state->_calib_dt_CAMERAtoIMU->set_value(temp_cameraimu_dt);
    state->_calib_dt_CAMERAtoIMU->set_fe(temp_cameraimu_dt);

    /// Extrinsic Calibration
    state->_calib_CAMERAtoIMU->set_value(params.camera_imu_extrinsics);
    state->_calib_CAMERAtoIMU->set_fe(params.camera_imu_extrinsics);

    /// Propagator
    propagator = new Propagator(params.imu_noises, params.gravity);

    /// Initializer
    initializer = new InertialInitializer(params.gravity, params.init_window_time, params.init_imu_thresh);

    /// Make the Updater
    updaterCameraTracking = new UpdaterCameraTracking(params.updaterOptions);
    /// Initialize the Camera Pose Object
    cameraPoseTracker = std::make_shared<camimucalib_core::cameraPoseTracking>(params.checkerboard_dx, params.checkerboard_dy,
                                                                               params.checkerboard_rows, params.checkerboard_cols,
                                                                               params.camera_calibration_file_path);
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
    logData();
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

    bool boarddetected = cameraPoseTracker->feedImage(timestamp, image_in);

    bool did_propagate_update = do_propagate_update(timestamp, boarddetected);

    if(state->_clones_IMU.size() == 1) {
        /// G_T_I1
        Eigen::Matrix<double, 4, 1> q_GtoI1 = state->_imu->quat();
        Eigen::Matrix3d I1_R_G = camimucalib_core::quat_2_Rot(q_GtoI1);
        Eigen::Matrix3d G_R_I1 = I1_R_G.transpose();
        Eigen::Vector3d G_t_I1 = state->_imu->pos();
        G_T_I1.block(0, 0, 3, 3) = G_R_I1;
        G_T_I1.block(0, 3, 3, 1) = G_t_I1;
    }
    Eigen::Matrix4d G_T_Ik = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d Ik_R_G = camimucalib_core::quat_2_Rot(state->_imu->quat());
    Eigen::Matrix3d G_R_Ik = Ik_R_G.transpose();
    Eigen::Vector3d G_t_Ik = state->_imu->pos();
    G_T_Ik.block(0, 0, 3, 3) = G_R_Ik;
    G_T_Ik.block(0, 3, 3, 1) = G_t_Ik;
    Eigen::Matrix4d I1_T_Ik = G_T_I1.inverse()*G_T_Ik;
    Pose *calibration = state->_calib_CAMERAtoIMU;
    Eigen::Matrix3d I_R_C = calibration->Rot();
    Eigen::Vector3d I_t_C = calibration->pos();
    Eigen::Matrix4d I_T_C = Eigen::Matrix4d::Identity();
    I_T_C.block(0, 0, 3, 3) = I_R_C;
    I_T_C.block(0, 3, 3, 1) = I_t_C;
    repErr = cameraPoseTracker->checkReprojections(I_T_C, I1_T_Ik);
    /// Printing for debug
    logData();
}

bool camimucalib_estimator::camimucalibManager::do_propagate_update(double timestamp, bool boarddetected) {
    if(state->_timestamp >= timestamp) {
        printf(YELLOW "Stepping back in time!!! (prop dt = %3f)\n" RESET, (timestamp-state->_timestamp));
        return false;
    }
    /// Propagate the state forward to the current update time
    /// Also augment it with a clone!
    propagator->propagate_and_clone(state, timestamp);
    /// Return if we are unable to propagate
    if (state->_timestamp != timestamp) {
        printf(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
        printf(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET,timestamp-state->_timestamp);
        return false;
    }
    if(state->_clones_IMU.size() < 2) {
        printf(YELLOW "[camimucalib_estimator::camimucalibManager::do_propagate_update] state->_clones_IMU.size() must be > 2\n");
        return false;
    }
    std::cout << "Board Detected ?: " << boarddetected << std::endl;
    if (boarddetected) {
        /// Marginalize the oldest clone if needed
        if(did_update1 && did_update2) {
            StateHelper::marginalize_old_clone(state);
        }
        relativePose rP = cameraPoseTracker->getRelativePose();
        updaterCameraTracking->updateImage2Image(state, rP, did_update1);
        Eigen::Matrix4d Im1_T_Imk = cameraPoseTracker->getCameraPose().pose;
        updaterCameraTracking->updateImage2FirstImage(state, Im1_T_Imk, G_T_I1, timestamp, did_update2);
        if(did_update1 && did_update2)
            return true;
    }
    return false;
}

void camimucalib_estimator::camimucalibManager::logData() {
//    std::cout << YELLOW << "Started Printing" << std::endl;
    Pose* calib = state->_calib_CAMERAtoIMU;
    Eigen::Matrix3d I_R_G = camimucalib_core::quat_2_Rot(state->_imu->quat());
    Eigen::Matrix3d G_R_I = I_R_G.transpose();
    Eigen::Quaterniond G_quat_I(G_R_I);

    /// 1
    std::vector<Type*> statevars_pose;
    statevars_pose.push_back(state->_imu->pose());
    Eigen::Matrix<double, 6, 6> covariance_imu_pose = StateHelper::get_marginal_covariance(state, statevars_pose);
    trajfile_csv << G_quat_I.x() << ", " << G_quat_I.y() << ", " << G_quat_I.z() << ", " << G_quat_I.w() << ", "
                 << state->_imu->pos().x() << ", " << state->_imu->pos().y() << ", " << state->_imu->pos().z()  << std::endl;

    /// 2
    std::vector<Type*> statevars_bias_a;
    statevars_bias_a.push_back(state->_imu->ba());
    Eigen::Matrix<double, 3, 3> covariance_imu_ba = StateHelper::get_marginal_covariance(state, statevars_bias_a);
    std::vector<Type*> statevars_bias_g;
    statevars_bias_g.push_back(state->_imu->bg());
    Eigen::Matrix<double, 3, 3> covariance_imu_bg = StateHelper::get_marginal_covariance(state, statevars_bias_g);
    bias_csv << state->_imu->bias_a().x() << ", " << state->_imu->bias_a().y() << ", " << state->_imu->bias_a().z() << ", "
             << state->_imu->bias_g().x() << ", " << state->_imu->bias_g().y() << ", " << state->_imu->bias_g().z() << ", "
             << sqrt(covariance_imu_ba(0, 0)) << ", " << sqrt(covariance_imu_ba(1, 1)) << ", " << sqrt(covariance_imu_ba(2, 2)) << ", "
             << sqrt(covariance_imu_bg(0, 0)) << ", " << sqrt(covariance_imu_bg(1, 1)) << ", " << sqrt(covariance_imu_bg(2, 2)) << std::endl;

    /// 3
    std::vector<Type*> statevars_velocity;
    statevars_velocity.push_back(state->_imu->v());
    Eigen::Matrix<double, 3, 3> covariance_imu_velocity = StateHelper::get_marginal_covariance(state, statevars_velocity);
    velocity_csv << state->_imu->vel().x() << ", " << state->_imu->vel().y() << ", " << state->_imu->vel().z() << ", "
                 << sqrt(covariance_imu_velocity(0, 0)) << ", "<< sqrt(covariance_imu_velocity(1, 1)) << ", "<< sqrt(covariance_imu_velocity(2, 2)) << std::endl;

    /// 4
    std::vector<Type*> statevars_calib_extrinsic;
    statevars_calib_extrinsic.push_back(state->_calib_CAMERAtoIMU);
    Eigen::Matrix<double, 6, 6> covariance_calib_extrinsic = StateHelper::get_marginal_covariance(state, statevars_calib_extrinsic);
    calib_extrinsic_csv << calib->quat()(0) << "," << calib->quat()(1) << ", " << calib->quat()(2) << ", " << calib->quat()(3) << ", "
                        << calib->pos()(0) << "," << calib->pos()(1) << "," << calib->pos()(2) << ", "
                        << sqrt(covariance_calib_extrinsic(0, 0)) << ", " << sqrt(covariance_calib_extrinsic(1, 1)) << ", " << sqrt(covariance_calib_extrinsic(2, 2)) << ", "
                        << sqrt(covariance_calib_extrinsic(3, 3)) << ", " << sqrt(covariance_calib_extrinsic(4, 4)) << ", " << sqrt(covariance_calib_extrinsic(5, 5)) << std::endl;

//    std::cout << "Crashed here 1!" << std::endl;
//    /// 5
//    std::vector<Type*> statevars_calib_dt;
//    statevars_calib_dt.push_back(state->_calib_dt_CAMERAtoIMU);
//    std::cout << "Crashed here 2!" << std::endl;
//    Eigen::Matrix<double, 1, 1> covariance_calib_dt = StateHelper::get_marginal_covariance(state, statevars_calib_dt);
//    std::cout << "Crashed here 3!" << std::endl;
//    calib_dt_csv << state->_calib_dt_CAMERAtoIMU->value()(0) << ", " << sqrt(covariance_calib_dt(0, 0)) << std::endl;
////    std::cout << GREEN << "Time Delay: " << 1000*state->_calib_dt_LIDARtoIMU->value()(0) << " [ms]" << std::endl;
////    std::cout << YELLOW << "Done Printing" << std::endl;
//    std::cout << "Crashed here 5!" << std::endl;

    /// 6
    cameraPoseTracking::Odom camPose = cameraPoseTracker->getCameraPose();
    Eigen::Matrix4d C0_T_Ck = camPose.pose;
    Eigen::Matrix3d C0_R_Ck = C0_T_Ck.block(0, 0, 3, 3);
//    std::cout << "C0_T_Ck: \n" << std::endl;
//    std::cout << C0_T_Ck << std::endl << std::endl;
    Eigen::Quaterniond C0_quat_Ck(C0_R_Ck);
    Eigen::Vector3d C0_t_Ck = C0_T_Ck.block(0, 3, 3, 1);
    visodom_csv << C0_quat_Ck.x() << ", " << C0_quat_Ck.y() << ", " << C0_quat_Ck.z() << ", " << C0_quat_Ck.w() << ", "
                 << C0_t_Ck.x() << ", " << C0_t_Ck.y() << ", "<< C0_t_Ck.z() << std::endl;

    ///
    repErr_csv << repErr << "\n";
}