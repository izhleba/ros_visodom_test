cmake_minimum_required(VERSION 2.8.3)
project(odometry_estimator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        roscpp
        rospy
        image_transport
        )


#find_package(catkin REQUIRED COMPONENTS
#        roscpp
#        sensor_msgs
#        nav_msgs
#        message_filters
#        image_transport
#        cv_bridge
#        image_geometry
#        tf
#        pcl_ros
#        std_srvs
#        message_generation
#        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)


add_executable(odometry_estimator src/odometry_estimator.cpp)
#add_dependencies(odometry_estimator nav_msgs_generate_messages_cpp)

target_link_libraries(odometry_estimator ${catkin_LIBRARIES})