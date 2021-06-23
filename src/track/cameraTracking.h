//
// Created by usl on 12/10/20.
//

#ifndef CAMIMUCALIB_CAMERATRACKING_H
#define CAMIMUCALIB_CAMERATRACKING_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "utils/quat_ops.h"
#include "utils/math_utils.h"
#include "utils/eigen_utils.h"

#include "types/Pose.h"
#include "relpose/relativePose.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <fstream>
#include <memory>

namespace camimucalib_core {
    class cameraTracking {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<cameraTracking> Ptr;
        struct Odom {
            double timestamp;
            Eigen::Matrix4d pose;
        };
        explicit  cameraTracking(int dx_ = 0.05, int dy_ = 0.05,
                       double checkerboard_rows_ = 6,
                       double checkerboard_cols_ = 7,
                       std::string cam_config_file_path_ = "");
        void readCameraParams(std::string cam_config_file_path,
                              int &image_height, int &image_width,
                              cv::Mat &D, cv::Mat &K);
        void feedImage(double timestamp, cv::Mat input_image,
                       Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity());
        void estimateCameraPose();
        Odom getCameraPose();
        relativePose getRelativePose();

    private:
        cv::Mat image_in;
        cv::Mat projection_matrix;
        cv::Mat distCoeff;
        int image_height, image_width;

        std::vector<cv::Point2f> image_points;
        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> projected_points;

        double dx, dy;
        int checkerboard_rows, checkerboard_cols;

        cv::Mat tvec, rvec;
        cv::Mat C_R_W;

        Eigen::Matrix3d C_R_W_eig;
        Eigen::Vector3d C_t_W_eig;
        Eigen::Matrix4d C_T_W_eig;
        Eigen::Matrix4d W_T_C_eig;
        Eigen::Matrix4d W_T_C_eig_first;

        Eigen::Matrix4d camera2ros;
        std::string cam_config_file_path;

        bool first_frame;
        double current_timestamp;
        double previous_timestamp;

        Odom currentpose;

        Eigen::Matrix4d C0_T_Ck;
        Eigen::Matrix4d C0_T_Ck_1;

        relativePose latestRP; // latest relative pose
    };
}

#endif //CAMIMUCALIB_CAMERATRACKING_H
