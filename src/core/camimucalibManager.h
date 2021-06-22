//
// Created by usl on 12/8/20.
//

#ifndef CAMIMUCALIB_ESTIMATOR_CAMIMUCALIBMANAGER_H
#define CAMIMUCALIB_ESTIMATOR_CAMIMUCALIBMANAGER_H

#include <string>
#include <algorithm>
#include <fstream>
#include <Eigen/StdVector>
#include <boost/filesystem.hpp>

#include "init/InertialInitializer.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "state/Propagator.h"
#include "update/UpdaterCameraTracking.h"
#include "track/cameraTracking.h"

#include "camimucalibManagerOptions.h"

namespace camimucalib_estimator {
    /// Core class that manages the entire system
    class camimucalibManager {
    public:
        /// Constructor that will load all configuration variables
        camimucalibManager(camimucalibManagerOptions& param_);

        /// Feed function for inertial data
        void feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am);

        /// Feed function for camera data
        void feed_measurement_camera(double timestamp, cv::Mat image_in);

        /// If we initialized or not
        bool initialized() {
            return is_initialized_camimucalib;
        }

        /// Timestamp the system was initialized at
        double initialized_time() {
            return startup_time;
        }

        /// Accessor to get the current state
        State* get_state() {
            return state;
        }

        /// Accessor to get the current propagator
        Propagator* get_propagator() {
            return propagator;
        }

//        /// Accessor to Camera Odometry object
//        camimucalib_core::cameraTracking::Ptr get_track_lodom() {
//            return cameraPoseTracker;
//        }

        /// Returns the last timestamp we have marginalized (true if we have a state)
        bool hist_last_marg_state(double &timestamp, Eigen::Matrix<double,7,1> &stateinG) {
            if(hist_last_marginalized_time != -1) {
                timestamp = hist_last_marginalized_time;
                stateinG = hist_stateinG.at(hist_last_marginalized_time);
                return true;
            } else {
                timestamp = -1;
                stateinG.setZero();
                return false;
            }
        }

        /// This function will try to initialize the state
        /// This function could also be repurposed to re-initialize the system after failure
        bool try_to_initialize();

        /// Boolean if we are initialized or not
        bool is_initialized_camimucalib = false;

        /// G_T_I1;
        Eigen::Matrix4d G_T_I1 = Eigen::Matrix4d::Identity();

        /// Print State for debugging
        void printState();


    protected:

        /// This will do propagation and updates
        void do_propagate_update(double timestamp);

        ///The following will update our historical tracking information
        void update_keyframe_historical_information();

        /// Manager of parameters
        camimucalibManagerOptions params;

        /// Our master state object
        State* state;

        /// Propagator of our state
        Propagator* propagator;

        /// State initializer
        InertialInitializer* initializer;

        /// HEC Updater
        UpdaterCameraTracking* updaterCameraTracking;

        /// Camera Pose object (Tracker)
//        camimucalib_core::cameraTracking::Ptr cameraPoseTracker;

        /// Track the distance travelled
        double timelastupdate = -1;
        double distance = 0;

        /// Start-up time of the filter
        double startup_time = -1;

        /// Historical information of the filter
        double hist_last_marginalized_time = -1;
        std::map<double, Eigen::Matrix<double, 7, 1> > hist_stateinG;

        std::ofstream trajfile_csv;
        std::ofstream bias_csv;
        std::ofstream velocity_csv;
        std::ofstream calib_extrinsic_csv;
        std::ofstream calib_dt_csv;

        /// Update flags
        bool did_update1 = false, did_update2 = false;
    };
}
#endif //CAMIMUCALIB_ESTIMATOR_CAMIMUCALIBMANAGER_H
