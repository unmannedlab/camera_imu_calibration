//
// Created by usl on 11/29/20.
//

#ifndef CAMIMUCALIB_CORE_RELATIVEPOSE_H
#define CAMIMUCALIB_CORE_RELATIVEPOSE_H

#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include "types/Pose.h"

using namespace camimucalib_type;

namespace camimucalib_core {
    class relativePose {
    public:
        /// Time stamp of scan i
        double timestamp_i;
        /// Time stamp of scan j
        double timestamp_j;
        /// Odometry pose
        Eigen::Matrix4d odometry_ij;
    };
}
#endif //LIN_CORE_RELATIVEPOSE_H
