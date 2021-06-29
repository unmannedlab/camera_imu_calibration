#include "cameraTracking.h"

namespace camimucalib_core {

    cameraTracking::cameraTracking(int dx_, int dy_,
                                   double checkerboard_rows_,
                                   double checkerboard_cols_,
                                   std::string cam_config_file_path_) {
        cam_config_file_path = cam_config_file_path_;
        dx = dx_;
        dy = dy_;
        checkerboard_rows = checkerboard_rows_;
        checkerboard_cols = checkerboard_cols_;

        projection_matrix = cv::Mat::zeros(3, 3, CV_64F);
        distCoeff = cv::Mat::zeros(5, 1, CV_64F);

        readCameraParams(cam_config_file_path, image_height, image_width, distCoeff, projection_matrix);

        for(int i = 0; i < checkerboard_rows; i++)
            for (int j = 0; j < checkerboard_cols; j++)
                object_points.emplace_back(cv::Point3f(i*dx, j*dy, 0.0));

        first_frame = true;
        camera2ros << 0,  0, 1, 0,
                     -1,  0, 0, 0,
                      0, -1, 0, 0,
                      0,  0, 0, 1;
        C0_T_Ck_1 = Eigen::Matrix4d::Identity();
    }

    void cameraTracking::readCameraParams(std::string cam_config_file_path,
                                          int &image_height,
                                          int &image_width,
                                          cv::Mat &D,
                                          cv::Mat &K) {
        cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
        if(!fs_cam_config.isOpened())
            std::cerr << "Error: Wrong path: " << cam_config_file_path << std::endl;
        fs_cam_config["image_height"] >> image_height;
        fs_cam_config["image_width"] >> image_width;
        std::cout << "image_height: " << image_height << std::endl;
        std::cout << "image_width: " << image_width << std::endl;
        fs_cam_config["k1"] >> D.at<double>(0);
        fs_cam_config["k2"] >> D.at<double>(1);
        fs_cam_config["p1"] >> D.at<double>(2);
        fs_cam_config["p2"] >> D.at<double>(3);
        fs_cam_config["k3"] >> D.at<double>(4);
        fs_cam_config["fx"] >> K.at<double>(0, 0);
        fs_cam_config["fy"] >> K.at<double>(1, 1);
        fs_cam_config["cx"] >> K.at<double>(0, 2);
        fs_cam_config["cy"] >> K.at<double>(1, 2);
        C_T_W_eig = Eigen::Matrix4d::Identity();
        W_T_C_eig = Eigen::Matrix4d::Identity();
    }

    void cameraTracking::feedImage(double timestamp, cv::Mat input_image, Eigen::Matrix4d pose_predict) {
        current_timestamp = timestamp;
        image_in = input_image;
        bool boardDetectedInCam = cv::findChessboardCorners(image_in, cv::Size(checkerboard_cols, checkerboard_rows),
                                                            image_points,cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
                                                                         cv::CALIB_CB_FAST_CHECK);
        cv::drawChessboardCorners(image_in, cv::Size(checkerboard_cols, checkerboard_rows), image_points, boardDetectedInCam);
        if(boardDetectedInCam) {
            cv::cornerSubPix(image_in, image_points, cv::Size(21, 21),
                             cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            assert(image_points.size() == object_points.size());
            estimateCameraPose();
        }
    }

    void cameraTracking::estimateCameraPose() {
        cv::solvePnP(object_points, image_points, projection_matrix, distCoeff, rvec, tvec, false, CV_ITERATIVE);
        projected_points.clear();
        cv::projectPoints(object_points, rvec, tvec, projection_matrix, distCoeff, projected_points, cv::noArray());
        for(int i = 0; i < projected_points.size(); i++){
            cv::circle(image_in, projected_points[i], 3, cv::Scalar(0, 255, 0), -1, cv::LINE_AA, 0);
        }
        cv::Rodrigues(rvec, C_R_W);
        cv::cv2eigen(C_R_W, C_R_W_eig);
        C_t_W_eig = Eigen::Vector3d(tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2));

        C_T_W_eig.block(0, 0, 3, 3) = C_R_W_eig;
        C_T_W_eig.block(0, 3, 3, 1) = C_t_W_eig;

        W_T_C_eig = C_T_W_eig.inverse();

        if (first_frame) {
            W_T_C_eig_first = W_T_C_eig;
        }

        C0_T_Ck = camera2ros*W_T_C_eig_first.inverse()*W_T_C_eig*camera2ros.inverse();
        Eigen::Matrix4d deltaPose = C0_T_Ck_1.inverse()*C0_T_Ck;

        if(!first_frame) {
            latestRP.timestamp_i = previous_timestamp;
            latestRP.timestamp_j = current_timestamp;
            latestRP.odometry_ij = deltaPose;
        }

        currentpose.timestamp = current_timestamp;
        currentpose.pose = C0_T_Ck;

        C0_T_Ck_1 = C0_T_Ck;
        previous_timestamp = current_timestamp;
        first_frame = false;
    }

    relativePose cameraTracking::getRelativePose() {
        return latestRP;
    }

    cameraTracking::Odom cameraTracking::getCameraPose() {
        return currentpose;
    }
}