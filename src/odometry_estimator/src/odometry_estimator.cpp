#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <boost/format.hpp>

#include "tf/tfMessage.h"

#include <tf/tf.h>

typedef message_filters::sync_policies::ApproximateTime <nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicyOO;

typedef message_filters::sync_policies::ApproximateTime <geometry_msgs::TransformStamped, nav_msgs::Odometry> MySyncPolicyTO;

typedef message_filters::sync_policies::ApproximateTime <nav_msgs::Odometry, geometry_msgs::TransformStamped> MySyncPolicyOT;

typedef message_filters::sync_policies::ApproximateTime <geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> MySyncPolicyTT;

ros::Publisher transformExpPub;
ros::Subscriber tfExpSub;

ros::Publisher transformActPub;
ros::Subscriber tfActSub;

double lastExpectedTime = -1;
tf::Vector3 lastExpectedPos;
tf::Quaternion lastExpectedRot;

double lastActualTime = -1;
tf::Vector3 lastActualPos;
tf::Quaternion lastActualRot;

std::ofstream outFile;

std::string actual_tf_frame_id;
std::string expected_tf_frame_id;

double posXErrSum = 0.0;
double posYErrSum = 0.0;
double posZErrSum = 0.0;
double orientXErrSum = 0.0;
double orientYErrSum = 0.0;
double orientZErrSum = 0.0;
double orientWErrSum = 0.0;

double counterPosX = 0;
double counterPosY = 0;
double counterPosZ = 0;
double counterOrientX = 0;
double counterOrientY = 0;
double counterOrientZ = 0;
double counterOrientW = 0;

geometry_msgs::TransformStampedPtr findTf(const tf::tfMessageConstPtr msg, std::string frame_id) {
    geometry_msgs::TransformStampedPtr ptr = nullptr;
    for (int i = 0; i < msg->transforms.size(); i++) {
        geometry_msgs::TransformStamped trans = msg->transforms[i];
        if (trans.header.frame_id.compare(frame_id) == 0) {
            ptr.reset(new geometry_msgs::TransformStamped(trans));
        }
    }
    return ptr;
}

void handleError(double expectedTime, tf::Vector3 expectedPos, tf::Quaternion expectedRot,
                 double actualTime, tf::Vector3 actualPos, tf::Quaternion actualRot) {
    double et = expectedTime;
    double at = actualTime;
    double epx = expectedPos.x();
    double epy = expectedPos.y();
    double epz = expectedPos.z();
    double eox = expectedRot.x();
    double eoy = expectedRot.y();
    double eoz = expectedRot.z();
    double eow = expectedRot.w();
    double apx = actualPos.x();
    double apy = actualPos.y();
    double apz = actualPos.z();
    double aox = actualRot.x();
    double aoy = actualRot.y();
    double aoz = actualRot.z();
    double aow = actualRot.w();
    // et,at,epx,epy,epz,eox,eoy,eoz,eow,apx,apy,apz,aox,aoy,aoz,aow
    std::string line = str(
            boost::format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n") % et % at % epx % epy % epz % eox % eoy %
            eoz %
            eow % apx % apy % apz % aox % aoy % aoz % aow);

    if (outFile) {
        outFile << line;
    }

    if (lastExpectedTime > 0) {
        double expDelta = et - lastExpectedTime;
        double actDelta = at - lastActualTime;


        double expDeltaX = epx - lastExpectedPos.x();
        double expDeltaY = epy - lastExpectedPos.y();
        double expDeltaZ = epz - lastExpectedPos.z();

        double actualDeltaX = apx - lastActualPos.x();
        double actualDeltaY = apy - lastActualPos.y();
        double actualDeltaZ = apz - lastActualPos.z();

        double errorX = 1 - actualDeltaX / expDeltaX;
        double errorY = 1 - actualDeltaY / expDeltaY;
        double errorZ = 1 - actualDeltaZ / expDeltaZ;

        double expDeltaOrientationX = eox - lastExpectedRot.x();
        double expDeltaOrientationY = eoy - lastExpectedRot.y();
        double expDeltaOrientationZ = eoz - lastExpectedRot.z();
        double expDeltaOrientationW = eoz - lastExpectedRot.w();

        double actualDeltaOrientationX = aox - lastActualRot.x();
        double actualDeltaOrientationY = aoy - lastActualRot.y();
        double actualDeltaOrientationZ = aoz - lastActualRot.z();
        double actualDeltaOrientationW = aow - lastActualRot.w();

        double errorOrientationX = 1 - actualDeltaOrientationX / expDeltaOrientationX;
        double errorOrientationY = 1 - actualDeltaOrientationY / expDeltaOrientationY;
        double errorOrientationZ = 1 - actualDeltaOrientationZ / expDeltaOrientationZ;
        double errorOrientationW = 1 - actualDeltaOrientationW / expDeltaOrientationW;

        ROS_DEBUG("Time dif: %f; Error (x,y,z,{w}) pos: %f,%f,%f; orient: %f,%f,%f,%f", std::abs(expDelta - actDelta),
                  errorX, errorY, errorZ, errorOrientationX, errorOrientationY, errorOrientationZ, errorOrientationW);

        if (std::isfinite(errorX)) {
            posXErrSum += errorX;
            counterPosX++;
        }
        if (std::isfinite(errorY)) {
            posYErrSum += errorY;
            counterPosY++;
        }
        if (std::isfinite(errorZ)) {
            posZErrSum += errorZ;
            counterPosZ++;
        }
        if (std::isfinite(errorOrientationX)) {
            orientXErrSum += errorOrientationX;
            counterOrientX++;
        }
        if (std::isfinite(errorOrientationY)) {
            orientYErrSum += errorOrientationY;
            counterOrientY++;
        }
        if (std::isfinite(errorOrientationZ)) {
            orientZErrSum += errorOrientationZ;
            counterOrientZ++;
        }
        if (std::isfinite(errorOrientationW)) {
            orientWErrSum += errorOrientationW;
            counterOrientW++;
        }


        ROS_INFO("Mean error: px:%f,py:%f,pz:%f,ox:%f,oy:%f,oz:%f,ow:%f", posXErrSum * 100 / counterPosX,
                 posYErrSum * 100 / counterPosY, posZErrSum * 100 / counterPosZ, orientXErrSum * 100 / counterOrientX,
                 orientYErrSum * 100 / counterOrientY, orientZErrSum * 100 / counterOrientZ,
                 orientWErrSum * 100 / counterOrientW);
    }
    lastExpectedTime = expectedTime;
    lastExpectedPos = expectedPos;
    lastExpectedRot = expectedRot;

    lastActualTime = actualTime;
    lastActualPos = actualPos;
    lastActualRot = actualRot;
}

void callbackTO(const geometry_msgs::TransformStampedConstPtr &expectedMsg,
                const nav_msgs::OdometryConstPtr &actualMsg) {
    double expectedTime = expectedMsg->header.stamp.toSec();
    double actualTime = actualMsg->header.stamp.toSec();
    tf::Vector3 expectedPos(expectedMsg->transform.translation.x,
                            expectedMsg->transform.translation.y,
                            expectedMsg->transform.translation.z);
    tf::Quaternion expectedRot(expectedMsg->transform.rotation.x,
                               expectedMsg->transform.rotation.y,
                               expectedMsg->transform.rotation.z,
                               expectedMsg->transform.rotation.w);
    tf::Vector3 actualPos(actualMsg->pose.pose.position.x,
                          actualMsg->pose.pose.position.y,
                          actualMsg->pose.pose.position.z);
    tf::Quaternion actualRot(actualMsg->pose.pose.orientation.x,
                             actualMsg->pose.pose.orientation.y,
                             actualMsg->pose.pose.orientation.z,
                             actualMsg->pose.pose.orientation.w);
    handleError(expectedTime, expectedPos, expectedRot, actualTime, actualPos, actualRot);
}

void callbackOO(const nav_msgs::OdometryConstPtr &expectedMsg,
                const nav_msgs::OdometryConstPtr &actualMsg) {
    double expectedTime = expectedMsg->header.stamp.toSec();
    double actualTime = actualMsg->header.stamp.toSec();
    tf::Vector3 expectedPos(expectedMsg->pose.pose.position.x,
                            expectedMsg->pose.pose.position.y,
                            expectedMsg->pose.pose.position.z);
    tf::Quaternion expectedRot(expectedMsg->pose.pose.orientation.x,
                               expectedMsg->pose.pose.orientation.y,
                               expectedMsg->pose.pose.orientation.z,
                               expectedMsg->pose.pose.orientation.w);
    tf::Vector3 actualPos(actualMsg->pose.pose.position.x,
                          actualMsg->pose.pose.position.y,
                          actualMsg->pose.pose.position.z);
    tf::Quaternion actualRot(actualMsg->pose.pose.orientation.x,
                             actualMsg->pose.pose.orientation.y,
                             actualMsg->pose.pose.orientation.z,
                             actualMsg->pose.pose.orientation.w);
    handleError(expectedTime, expectedPos, expectedRot, actualTime, actualPos, actualRot);
}

void callbackOT(const nav_msgs::OdometryConstPtr &expectedMsg,
                const geometry_msgs::TransformStampedConstPtr &actualMsg) {
    double expectedTime = expectedMsg->header.stamp.toSec();
    double actualTime = actualMsg->header.stamp.toSec();
    tf::Vector3 expectedPos(expectedMsg->pose.pose.position.x,
                            expectedMsg->pose.pose.position.y,
                            expectedMsg->pose.pose.position.z);
    tf::Quaternion expectedRot(expectedMsg->pose.pose.orientation.x,
                               expectedMsg->pose.pose.orientation.y,
                               expectedMsg->pose.pose.orientation.z,
                               expectedMsg->pose.pose.orientation.w);
    tf::Vector3 actualPos(actualMsg->transform.translation.x,
                          actualMsg->transform.translation.y,
                          actualMsg->transform.translation.z);
    tf::Quaternion actualRot(actualMsg->transform.rotation.x,
                             actualMsg->transform.rotation.y,
                             actualMsg->transform.rotation.z,
                             actualMsg->transform.rotation.w);
    handleError(expectedTime, expectedPos, expectedRot, actualTime, actualPos, actualRot);
}

void callbackTT(const geometry_msgs::TransformStampedConstPtr &expectedMsg,
                const geometry_msgs::TransformStampedConstPtr &actualMsg) {
    double expectedTime = expectedMsg->header.stamp.toSec();
    double actualTime = actualMsg->header.stamp.toSec();
    tf::Vector3 expectedPos(expectedMsg->transform.translation.x,
                            expectedMsg->transform.translation.y,
                            expectedMsg->transform.translation.z);
    tf::Quaternion expectedRot(expectedMsg->transform.rotation.x,
                               expectedMsg->transform.rotation.y,
                               expectedMsg->transform.rotation.z,
                               expectedMsg->transform.rotation.w);
    tf::Vector3 actualPos(actualMsg->transform.translation.x,
                          actualMsg->transform.translation.y,
                          actualMsg->transform.translation.z);
    tf::Quaternion actualRot(actualMsg->transform.rotation.x,
                             actualMsg->transform.rotation.y,
                             actualMsg->transform.rotation.z,
                             actualMsg->transform.rotation.w);
    handleError(expectedTime, expectedPos, expectedRot, actualTime, actualPos, actualRot);
}

void tfExpCallback(const tf::tfMessage::ConstPtr &msg) {
    geometry_msgs::TransformStampedPtr transform = findTf(msg, expected_tf_frame_id);
    if (transform) {
        transformExpPub.publish(transform);
    } else {
        ROS_ERROR_STREAM("Cant find frame_id: " << expected_tf_frame_id);
    }
}

void tfActCallback(const tf::tfMessage::ConstPtr &msg) {
    geometry_msgs::TransformStampedPtr transform = findTf(msg, actual_tf_frame_id);
    if (transform) {
        transformActPub.publish(transform);
    } else {
        ROS_ERROR_STREAM("Cant find frame_id: " << actual_tf_frame_id);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_estimator");
    ros::NodeHandle nodeHandle;
    ROS_INFO_STREAM("Odometry_estimator is alive");

    ros::NodeHandle local_nh("~");
//        read params
    std::string expectedTopic;
    std::string actualTopic;
    double agePenalty;
    std::string outFilePath;

    local_nh.param("expected", expectedTopic, std::string("/expected"));
    local_nh.param("actual", actualTopic, std::string("/actual"));
    local_nh.param("age_penalty", agePenalty, 1.0);
    local_nh.param("out", outFilePath, std::string("/tmp/odometry.csv"));
    bool acTfFl = local_nh.getParam("actual_tf_frame_id", actual_tf_frame_id);
    bool exTfFl = local_nh.getParam("expected_tf_frame_id", expected_tf_frame_id);

    outFile = std::ofstream(outFilePath);
    if (outFile.is_open()) {
        // header
        outFile << "et,at,epx,epy,epz,eox,eoy,eoz,eow,apx,apy,apz,aox,aoy,aoz,aow\n";
    } else {
        ROS_ERROR_STREAM("Unable to open file:" << outFilePath);
    }
    //    ROS_DEBUG_STREAM("Args:" << expectedTopic << "," << actualTopic);

    if (exTfFl == false && acTfFl == false) {
        message_filters::Subscriber <nav_msgs::Odometry> subExpected(nodeHandle, expectedTopic, 1);
        message_filters::Subscriber <nav_msgs::Odometry> subActual(nodeHandle, actualTopic, 1);

        message_filters::Synchronizer <MySyncPolicyOO> sync(MySyncPolicyOO(10), subExpected, subActual);
        sync.setAgePenalty(agePenalty);
        sync.registerCallback(boost::bind(&callbackOO, _1, _2));

        ros::spin();
    } else if (exTfFl == true && acTfFl == false) {
        std::string newExpectedTopic = expectedTopic + "/" + expected_tf_frame_id;
        tfExpSub = nodeHandle.subscribe(expectedTopic, 1000, tfExpCallback);
        transformExpPub = nodeHandle.advertise<geometry_msgs::TransformStamped>(newExpectedTopic, 1000);

        message_filters::Subscriber <geometry_msgs::TransformStamped> subExpected(nodeHandle, newExpectedTopic, 1);
        message_filters::Subscriber <nav_msgs::Odometry> subActual(nodeHandle, actualTopic, 1);

        message_filters::Synchronizer <MySyncPolicyTO> sync(MySyncPolicyTO(10), subExpected, subActual);
        sync.setAgePenalty(agePenalty);
        sync.registerCallback(boost::bind(&callbackTO, _1, _2));

        ros::spin();
    } else if (exTfFl == false && acTfFl == true) {
        std::string newActualTopic = actualTopic + "/" + actual_tf_frame_id;
        tfActSub = nodeHandle.subscribe(actualTopic, 1000, tfActCallback);
        transformActPub = nodeHandle.advertise<geometry_msgs::TransformStamped>(newActualTopic, 1000);

        message_filters::Subscriber <nav_msgs::Odometry> subExpected(nodeHandle, expectedTopic, 1);
        message_filters::Subscriber <geometry_msgs::TransformStamped> subActual(nodeHandle, newActualTopic, 1);

        message_filters::Synchronizer <MySyncPolicyOT> sync(MySyncPolicyOT(10), subExpected, subActual);
        sync.setAgePenalty(agePenalty);
        sync.registerCallback(boost::bind(&callbackOT, _1, _2));

        ros::spin();
    } else {
        std::string newExpectedTopic = expectedTopic + "/" + expected_tf_frame_id;
        tfExpSub = nodeHandle.subscribe(expectedTopic, 1000, tfExpCallback);
        transformExpPub = nodeHandle.advertise<geometry_msgs::TransformStamped>(newExpectedTopic, 1000);

        std::string newActualTopic = actualTopic + "/" + actual_tf_frame_id;
        tfActSub = nodeHandle.subscribe(actualTopic, 1000, tfActCallback);
        transformActPub = nodeHandle.advertise<geometry_msgs::TransformStamped>(newActualTopic, 1000);


        message_filters::Subscriber <geometry_msgs::TransformStamped> subExpected(nodeHandle, newExpectedTopic, 1);
        message_filters::Subscriber <geometry_msgs::TransformStamped> subActual(nodeHandle, newActualTopic, 1);

        message_filters::Synchronizer <MySyncPolicyTT> sync(MySyncPolicyTT(10), subExpected, subActual);
        sync.setAgePenalty(agePenalty);
        sync.registerCallback(boost::bind(&callbackTT, _1, _2));

        ros::spin();
    }
//    ros::spin();

    outFile.close();
    return 0;
}
