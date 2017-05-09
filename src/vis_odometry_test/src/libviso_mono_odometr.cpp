
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "../libviso2/viso_mono.h"
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include "odometer_base.h"
#include "libviso_mono_odometr.h"

#include "utils.h"


LibvisoMonoOdometer::LibvisoMonoOdometer(const ros::NodeHandle &_nodeHandle, const std::string &imageTopic,
                                         const std::string &infoTopic, const std::string &odomOutTopic) :
        OdometerBase(_nodeHandle, imageTopic, infoTopic, odomOutTopic) {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    loadParams(local_nh, visualOdometerParams);
}

void LibvisoMonoOdometer::imageAnInfoCallback(
        const sensor_msgs::ImageConstPtr &imageMsg,
        const sensor_msgs::CameraInfoConstPtr &infoMsg) {
    ros::WallTime startTime = ros::WallTime::now();

    bool firstRun = false;
    // create odometer if not exists
    if (!visualOdometer) {
        firstRun = true;
        // fill params
        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(infoMsg);
        visualOdometerParams.calib.f = model.fx();
        visualOdometerParams.calib.cu = model.cx();
        visualOdometerParams.calib.cv = model.cy();
        ROS_INFO_STREAM("Model camera: fx:" << visualOdometerParams.calib.f << ", cx:" << visualOdometerParams.calib.cu
                                            << ", cy:" << visualOdometerParams.calib.cv);
        visualOdometer.reset(new VisualOdometryMono(visualOdometerParams));
        ROS_INFO_STREAM("Initialized libviso2 mono odometry "
                                "with the following parameters:" << std::endl <<
                                                                 visualOdometerParams);
    }

    // convert image if necessary
    uint8_t *imageData;
    int step;
    cv_bridge::CvImageConstPtr cv_ptr;
    if (imageMsg->encoding == sensor_msgs::image_encodings::MONO8) {
        imageData = const_cast<uint8_t *>(&(imageMsg->data[0]));
        step = imageMsg->step;
    } else {
        cv_ptr = cv_bridge::toCvShare(imageMsg, sensor_msgs::image_encodings::MONO8);
        imageData = cv_ptr->image.data;
        step = cv_ptr->image.step[0];
    }

    // run the odometer
    int32_t dims[] = {imageMsg->width, imageMsg->height, step};
    // on first run, only feed the odometer with first image pair without
    // retrieving data
    if (firstRun) {
        visualOdometer->process(imageData, dims);
        tf::Transform deltaTransform;
        deltaTransform.setIdentity();
        integrateAndPublish(deltaTransform, imageMsg->header.stamp);
    } else {
        bool success = visualOdometer->process(imageData, dims);
        if (success) {
            Matrix cameraMotion = Matrix::inv(visualOdometer->getMotion());
            ROS_DEBUG("Found %i matches with %i inliers.",
                      visualOdometer->getNumberOfMatches(),
                      visualOdometer->getNumberOfInliers());
            ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << cameraMotion);

            tf::Matrix3x3 rotationMat(
                    cameraMotion.val[0][0], cameraMotion.val[0][1], cameraMotion.val[0][2],
                    cameraMotion.val[1][0], cameraMotion.val[1][1], cameraMotion.val[1][2],
                    cameraMotion.val[2][0], cameraMotion.val[2][1], cameraMotion.val[2][2]);
            tf::Vector3 translationVec(cameraMotion.val[0][3], cameraMotion.val[1][3], cameraMotion.val[2][3]);
            tf::Transform deltaTransform(rotationMat, translationVec);
            integrateAndPublish(deltaTransform, imageMsg->header.stamp);
        } else {
            ROS_DEBUG("Call to VisualOdometryMono::process() failed. Assuming motion too small.");
            tf::Transform deltaTransform;
            deltaTransform.setIdentity();
            integrateAndPublish(deltaTransform, imageMsg->header.stamp);
        }
    }
}

void LibvisoMonoOdometer::loadParams(const ros::NodeHandle &local_nh, Matcher::parameters &params) {
    local_nh.getParam("nms_n", params.nms_n);
    local_nh.getParam("nms_tau", params.nms_tau);
    local_nh.getParam("match_binsize", params.match_binsize);
    local_nh.getParam("match_radius", params.match_radius);
    local_nh.getParam("match_disp_tolerance", params.match_disp_tolerance);
    local_nh.getParam("outlier_disp_tolerance", params.outlier_disp_tolerance);
    local_nh.getParam("outlier_flow_tolerance", params.outlier_flow_tolerance);
    local_nh.getParam("multi_stage", params.multi_stage);
    local_nh.getParam("half_resolution", params.half_resolution);
    local_nh.getParam("refinement", params.refinement);
}


void LibvisoMonoOdometer::loadParams(const ros::NodeHandle &local_nh, VisualOdometry::bucketing &bucketing) {
    local_nh.getParam("max_features", bucketing.max_features);
    local_nh.getParam("bucket_width", bucketing.bucket_width);
    local_nh.getParam("bucket_height", bucketing.bucket_height);
}


void LibvisoMonoOdometer::loadCommonParams(const ros::NodeHandle &local_nh, VisualOdometry::parameters &params) {
    loadParams(local_nh, params.match);
    loadParams(local_nh, params.bucket);
}


void LibvisoMonoOdometer::loadParams(const ros::NodeHandle &local_nh, VisualOdometryMono::parameters &params) {
    loadCommonParams(local_nh, params);
    if (!local_nh.getParam("camera_height", params.height)) {
        ROS_WARN("Parameter 'camera_height' is required but not set. Using default: %f", params.height);
    }
    if (!local_nh.getParam("camera_pitch", params.pitch)) {
        ROS_WARN("Paramter 'camera_pitch' is required but not set. Using default: %f", params.pitch);
    }
    local_nh.getParam("ransac_iters", params.ransac_iters);
    local_nh.getParam("inlier_threshold", params.inlier_threshold);
    local_nh.getParam("motion_threshold", params.motion_threshold);
}
