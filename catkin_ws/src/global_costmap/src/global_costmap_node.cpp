#include <ros/ros.h>
#include <global_costmap.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_costmap_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Prius::GlobalCostmap cmap(nh, pnh, argv[1]);

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
