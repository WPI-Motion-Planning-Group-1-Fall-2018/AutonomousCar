#include <ros/ros.h>
#include <local_costmap.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_costmap_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Prius::LocalCostmap cmap(nh, pnh);

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
