cmake_minimum_required(VERSION 2.8.3)
project(vis_odometry_test)

# Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        nav_msgs
        message_filters
        image_transport
        cv_bridge
        image_geometry
        tf
        pcl_ros
        std_srvs
        message_generation
        )

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


#find_package(catkin REQUIRED COMPONENTS
#        nav_msgs
#        roscpp
#        rospy
#        image_transport
#        cv_bridge
#        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_library(libviso2 SHARED     libviso2/filter.cpp
                                libviso2/filter.h
                                libviso2/matcher.cpp
                                libviso2/matcher.h
                                libviso2/matrix.cpp
                                libviso2/matrix.h
                                libviso2/reconstruction.cpp
                                libviso2/reconstruction.h
                                libviso2/timer.h
                                libviso2/triangle.cpp
                                libviso2/triangle.h
                                libviso2/viso.cpp
                                libviso2/viso.h
                                libviso2/viso_mono.cpp
                                libviso2/viso_mono.h
                                libviso2/viso_stereo.cpp
                                libviso2/viso_stereo.h)


add_executable(odom_listener src/odom_listener.cpp src/libviso_mono_odometr.cpp)
target_link_libraries(odom_listener ${catkin_LIBRARIES} libviso2)
add_dependencies(odom_listener nav_msgs_generate_messages_cpp)

#target_link_libraries(odom_listener ${catkin_LIBRARIES})
target_link_libraries(odom_listener ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
