#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iostream>
#include "track/cameraPoseTracking.h"

#include "core/camimucalibManagerOptions.h"
#include "core/camimucalibManager.h"

#include "utils/parse_ros.h"
#include "utils/quat_ops.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

camimucalib_estimator::camimucalibManager *sys;

int main (int argc, char** argv) {
    ros::init(argc, argv, "ros_test_node");
    ros::NodeHandle nh("~");

    /// Create our camimucalib system
    camimucalib_estimator::camimucalibManagerOptions params = camimucalib_estimator::parse_ros_nodehandler(nh);
    sys = new camimucalib_estimator::camimucalibManager(params);
    /// Our topics (IMU & CAMERA)
    std::string topic_imu;
    std::string topic_image;
    nh.param<std::string>("topic_imu", topic_imu, "/imu");
    nh.param<std::string>("topic_image", topic_image, "/image");

    /// Location of the ROS bag we want to read in
    std::string path_to_bag;
    nh.param<std::string>("path_bag", path_to_bag, "/home/usl/Downloads/kalibr_calibration/Subodh_imucam_2021-06-17-19-15-37.bag");
    ROS_INFO_STREAM("ROS BAG PATH is: " << path_to_bag.c_str());

    /// Get our start location and how much of the bag we want to play
    /// Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO_STREAM("bag start: " << bag_start);
    ROS_INFO_STREAM("bag duration: " << bag_durr);

    /// Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    /// We should load the bag as a view
    /// Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    /// Start a few seconds in from the full view time
    /// If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish =
            (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO_STREAM("Time start = " << time_init.toSec());
    ROS_INFO_STREAM("Time end = " << time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    /// Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR_STREAM("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    cv::VideoWriter video("/home/smishr30/Downloads/output_video_cincalib/outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1920,1200));
    for (const rosbag::MessageInstance& m : view) {
        if (!ros::ok())
            break;

        sensor_msgs::Imu::ConstPtr s_imu = m.instantiate<sensor_msgs::Imu>();
        if (s_imu != nullptr && m.getTopic() == topic_imu) {
            double time_imu = (*s_imu).header.stamp.toSec();
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << (*s_imu).angular_velocity.x, (*s_imu).angular_velocity.y, (*s_imu).angular_velocity.z;
            am << (*s_imu).linear_acceleration.x, (*s_imu).linear_acceleration.y, (*s_imu).linear_acceleration.z;
//            ROS_INFO_STREAM("Feeding imu measurement");
            sys->feed_measurement_imu(time_imu, wm, am);
        }

        sensor_msgs::Image::ConstPtr s_image = m.instantiate<sensor_msgs::Image>();
        if (s_image != nullptr && m.getTopic() == topic_image) {
            double time_lidar = (*s_image).header.stamp.toSec();
            cv::Mat image_in = cv_bridge::toCvShare(s_image, "mono8")->image;
            cv::Mat image_in_color;
            cv::cvtColor(image_in, image_in_color, cv::COLOR_GRAY2BGR);
//            ROS_INFO_STREAM("Feeding camera measurement");
            sys->feed_measurement_camera(time_lidar, image_in_color);
            cv::Mat image_out;
            cv::resize(image_in_color, image_out, cv::Size(), 0.5, 0.5);
            cv::imshow("Image", image_in_color);
            video.write(image_in_color);

            cv::waitKey(10);
        }
    }
    video.release();

    ROS_INFO_STREAM("Reached end of bag");
    /// Write the final I_T_C1 to a text file
    Eigen::Matrix4d C1_T_C2;
    C1_T_C2 << 0,  0, 1, 0,
              -1,  0, 0, 0,
               0, -1, 0, 0,
               0,  0, 0, 1;
    camimucalib_estimator::State* state = sys->get_state();
    Pose *calibration = state->_calib_CAMERAtoIMU;
    Eigen::Matrix3d I_R_C1 = calibration->Rot();
    Eigen::Vector3d I_t_C1 = calibration->pos();
    Eigen::Matrix4d I_T_C1 = Eigen::Matrix4d::Identity();
    I_T_C1.block(0, 0, 3, 3) = I_R_C1;
    I_T_C1.block(0, 3, 3, 1) = I_t_C1;
    Eigen::Matrix4d I_T_C2 = I_T_C1*C1_T_C2;
//    std::cout << "I_T_C1: \n" << I_T_C1 << std::endl;
//    std::cout << "I_T_C: \n" << I_T_C2 << std::endl;
    std::cout << "Writing KF calibration result to: " << params.calibration_result_filename << std::endl;
    std::ofstream result_calibration;
    result_calibration.open(params.calibration_result_filename.c_str());
    result_calibration << I_T_C1;
    result_calibration.close();
    return EXIT_SUCCESS;
}