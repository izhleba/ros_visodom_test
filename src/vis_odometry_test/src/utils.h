#ifndef PROJECT_UTILS_H
#define PROJECT_UTILS_H


//double get_translation_error(const cv::Mat &t_true, const cv::Mat &t);
//
//
//double get_rotation_error(const cv::Mat &R_true, const cv::Mat &R);
//
//
//cv::Mat rot2euler(const cv::Mat & rotationMatrix);

//tf::Vector3 rot2euler(const tf::Matrix3x3 &rotationMatrix);

//// Computes the norm of the translation error
//double get_translation_error(const cv::Mat &t_true, const cv::Mat &t) {
//    return cv::norm(t_true - t);
//}
//
//// Computes the norm of the rotation error
//double get_rotation_error(const cv::Mat &R_true, const cv::Mat &R) {
//    cv::Mat error_vec, error_mat;
//    error_mat = R_true * cv::Mat(R.inv()).mul(-1);
//    cv::Rodrigues(error_mat, error_vec);
//
//    return cv::norm(error_vec);
//}

tf::Vector3 rotToEuler(const tf::Matrix3x3 &rotationMatrix) {
    double m00 = rotationMatrix[0][0];
    double m02 = rotationMatrix[0][2];
    double m10 = rotationMatrix[1][0];
    double m11 = rotationMatrix[1][1];
    double m12 = rotationMatrix[1][2];
    double m20 = rotationMatrix[2][0];
    double m22 = rotationMatrix[2][2];

    double x, y, z;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI / 2;
        z = atan2(m02, m22);
    } else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI / 2;
        z = atan2(m02, m22);
    } else {
        x = atan2(-m12, m11);
        y = asin(m10);
        z = atan2(-m20, m00);
    }

    tf::Vector3 euler(x, y, z);
    return euler;

}

#endif //PROJECT_UTILS_H

