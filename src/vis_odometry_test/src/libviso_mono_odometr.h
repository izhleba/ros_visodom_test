#ifndef LIBVISO_MONO_ODOMETR
#define LIBVISO_MONO_ODOMETR

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "../libviso2/viso_mono.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include "odometer_base.h"

class LibvisoMonoOdometer : public OdometerBase {

public:

    LibvisoMonoOdometer(const ros::NodeHandle &_nodeHandle, const std::string &imageTopic,
                        const std::string &infoTopic, const std::string &odomOutTopic);

protected:

    boost::shared_ptr<VisualOdometryMono> visualOdometer;
    VisualOdometryMono::parameters visualOdometerParams;

    void imageAnInfoCallback(const sensor_msgs::ImageConstPtr &imageMsg,
                             const sensor_msgs::CameraInfoConstPtr &infoMsg);

    void loadParams(const ros::NodeHandle &local_nh, Matcher::parameters &params);


    void loadParams(const ros::NodeHandle &local_nh, VisualOdometry::bucketing &bucketing);


    void loadCommonParams(const ros::NodeHandle &local_nh, VisualOdometry::parameters &params);


    void loadParams(const ros::NodeHandle &local_nh, VisualOdometryMono::parameters &params);
};

#endif //LIBVISO_MONO_ODOMETR