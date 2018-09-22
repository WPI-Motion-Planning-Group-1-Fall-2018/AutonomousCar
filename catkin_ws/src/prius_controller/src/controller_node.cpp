#include <ros/ros.h>
#include <controller.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behaviors_node");

  ros::NodeHandle nh;

  ros::NodeHandle pnh("~");

  Prius::PriusController cont(nh, pnh);

  ros::Rate rate(50);

  while(ros::ok)
  {
    ros::spinOnce();

    cont.calculateAndPublishControls();

    rate.sleep();

  }

}
