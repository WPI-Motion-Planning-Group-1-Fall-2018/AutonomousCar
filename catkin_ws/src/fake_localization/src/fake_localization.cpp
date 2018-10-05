#include <fake_localization.hpp>

namespace Prius {

FakeLocalization::FakeLocalization(ros::NodeHandle &nh)
{
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 100, &FakeLocalization::odometryCallback, this);
    joint_sub = nh.subscribe<sensor_msgs::JointState>("/unstamped/joint_states", 100, &FakeLocalization::jointStateCallback, this);
    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);
}

FakeLocalization::~FakeLocalization()
{
    ROS_INFO_STREAM("Fake Localization Destructor Called");
}

void FakeLocalization::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::StampedTransform transform;
    transform.stamp_ = msg->header.stamp;
    transform.frame_id_ = "map";
    transform.child_frame_id_ = "base_link";
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

    tf::Quaternion quat;
    quat.setW(msg->pose.pose.orientation.w);
    quat.setX(msg->pose.pose.orientation.x);
    quat.setY(msg->pose.pose.orientation.y);
    quat.setZ(msg->pose.pose.orientation.z);
    transform.setRotation(quat);

    br.sendTransform(transform);
}

void FakeLocalization::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    sensor_msgs::JointState joint_states = *msg;
    joint_states.header.stamp = ros::Time::now();
    joint_pub.publish(joint_states);
}
}
