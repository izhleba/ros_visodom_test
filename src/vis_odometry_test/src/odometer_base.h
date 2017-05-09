
#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>

class OdometerBase {

private:

    // optometry msg publisher
    //
    image_transport::Subscriber imageSub;
    //
    tf::Transform integratedPose;

    ros::Time lastUpdateTime;

    std::string odomFrameId;

    std::string baseLinkFrameId;

//    std::string sensorFrameId;

    tf::TransformListener tfListener;

    sensor_msgs::CameraInfoPtr lastInfoMsg;

    ros::NodeHandle nodeHandle;

    ros::Subscriber cameraInfoSub;

    ros::Publisher odomPub;

public:

    OdometerBase(const ros::NodeHandle &_nodeHandle, const std::string &imageTopic, const std::string &infoTopic,
                 const std::string &odomOutTopic) :
            nodeHandle(_nodeHandle),
            cameraInfoSub(nodeHandle.subscribe(infoTopic, 1, &OdometerBase::cameraInfoCallback, this)),
            odomPub(nodeHandle.advertise<nav_msgs::Odometry>(odomOutTopic, 1)) {

        ros::NodeHandle local_nh("~");
//        read params
        local_nh.param("odom_frame_id", odomFrameId, std::string("/odom"));
        local_nh.param("base_link_frame_id", baseLinkFrameId, std::string("/base_link"));
//        local_nh.param("sensor_frame_id", sensorFrameId, std::string("/camera"));
//        ROS_INFO_STREAM("baseLinkFrameId:" << baseLinkFrameId);

        image_transport::ImageTransport it(nodeHandle);

        imageSub = it.subscribe(imageTopic, 1, &OdometerBase::imageCallback, this);

        integratedPose.setIdentity();
    }

protected:

    virtual void imageAnInfoCallback(const sensor_msgs::ImageConstPtr &imageMsg,
                                     const sensor_msgs::CameraInfoConstPtr &infoMsg) = 0;

    void imageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
        if (lastInfoMsg) {
            imageAnInfoCallback(image_msg, lastInfoMsg);
        } else {
            ROS_DEBUG("Camera info not ready yet.");
        }
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg) {
        lastInfoMsg = boost::const_pointer_cast<sensor_msgs::CameraInfo>(info_msg);
    }

    void integrateAndPublish(const tf::Transform &deltaTransform, const ros::Time &timestamp) {
        if (timestamp < lastUpdateTime) {
            ROS_WARN("Negative time change!");
            return;
        }
        integratedPose *= deltaTransform;

        nav_msgs::Odometry odometryMsg;
        odometryMsg.header.stamp = timestamp;
        odometryMsg.header.frame_id = odomFrameId;
        odometryMsg.child_frame_id = baseLinkFrameId;
        tf::poseTFToMsg(integratedPose, odometryMsg.pose.pose);

        // calculate twist (not possible for first run as no deltaTime can be computed)
        if (!lastUpdateTime.isZero()) {
            double deltaTime = (timestamp - lastUpdateTime).toSec();
            if (deltaTime) {
                odometryMsg.twist.twist.linear.x = deltaTransform.getOrigin().getX() / deltaTime;
                odometryMsg.twist.twist.linear.y = deltaTransform.getOrigin().getY() / deltaTime;
                odometryMsg.twist.twist.linear.z = deltaTransform.getOrigin().getZ() / deltaTime;
                tf::Quaternion deltaRotation = deltaTransform.getRotation();
                tfScalar angle = deltaRotation.getAngle();
                tf::Vector3 axis = deltaRotation.getAxis();
                tf::Vector3 angularTwist = axis * angle / deltaTime;
                odometryMsg.twist.twist.angular.x = angularTwist.x();
                odometryMsg.twist.twist.angular.y = angularTwist.y();
                odometryMsg.twist.twist.angular.z = angularTwist.z();
            }
        }

        odomPub.publish(odometryMsg);

        lastUpdateTime = timestamp;
    }

};

#endif

