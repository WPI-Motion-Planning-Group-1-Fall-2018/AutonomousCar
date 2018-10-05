#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>


namespace Prius {

class FakeLocalization{

public:

    FakeLocalization(ros::NodeHandle &nh);
    ~FakeLocalization();

private:

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

    ros::Subscriber odom_sub;
    ros::Subscriber joint_sub;

    ros::Publisher joint_pub;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;

};

}
