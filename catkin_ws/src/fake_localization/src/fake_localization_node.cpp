#include <ros/ros.h>
#include <fake_localization.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_localization_node");

  ros::NodeHandle nh;

  Prius::FakeLocalization loc(nh);

  while(ros::ok)
  {
    ros::spin();
  }

  return 0;

}
