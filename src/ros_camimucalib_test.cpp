#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iostream>
#include "track/cameraTracking.h"

#include "core/camimucalibManagerOptions.h"
#include "core/camimucalibManager.h"

#include "utils/parse_ros.h"
#include "utils/quat_ops.h"

camimucalib_estimator::camimucalibManager *sys;

int main (int argc, char** argv) {
    ros::init(argc, argv, "ros_test_node");
    ros::NodeHandle nh("~");

    /// Create our camimucalib system
    camimucalib_estimator::camimucalibManagerOptions params = camimucalib_estimator::parse_ros_nodehandler(nh);
    sys = new camimucalib_estimator::camimucalibManager(params);

    /// Our topics (IMU & CAMERA)
    std::string topic_imu;
    std::string topic_lidar;
    nh.param<std::string>("topic_imu", topic_imu, "/imu");
    nh.param<std::string>("topic_lidar", topic_lidar, "/lidar");

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
    return EXIT_SUCCESS;
}