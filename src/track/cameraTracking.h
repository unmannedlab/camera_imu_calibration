//
// Created by usl on 12/10/20.
//

#ifndef CAMIMUCALIB_CAMERATRACKING_H
#define CAMIMUCALIB_CAMERATRACKING_H

#include "utils/quat_ops.h"
#include "utils/math_utils.h"
#include "utils/eigen_utils.h"

#include "types/Pose.h"
#include "relpose/relativePose.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <fstream>

namespace camimucalib_core {
    class cameraTracking {
        public:
        struct Odom {
            double timestamp;
            Eigen::Matrix4d pose;
        };
        cameraTracking();
        void feedImage(double timestamp,
                       cv::Mat input_image,
                       Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity());

    };
}

#endif //CAMIMUCALIB_CAMERATRACKING_H
