#pragma once

#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_listener.h>

namespace Prius {

class LocalCostmap{

public:

  LocalCostmap(ros::NodeHandle &nh, ros::NodeHandle pnh);

  ~LocalCostmap();

private:

  void centerLaserCallback(const sensor_msgs::PointCloud::ConstPtr &msg);

  void rightLaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

  void leftLaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

  void setupCostmap();

  void calcOccGrid();

  void mapPlanarScan(const sensor_msgs::LaserScan &scan, const std::string &frame);

  void mapPrevCenterScan();

  void mapCenterScan();

  int calcGridLocation(const double &x, const double &y);

  std::pair<double, double> calcCartesianCoords(const int &location);

  void inflateGrid();

  void clearMap();

  void pubOccGrid();

  sensor_msgs::LaserScan interpolatePlanarScan(const sensor_msgs::LaserScan &scan);

  ros::Subscriber center_laser_sub;

  ros::Subscriber right_laser_sub;

  ros::Subscriber left_laser_sub;

  ros::Publisher occ_grid_pub;

  tf::TransformListener listener;

  sensor_msgs::PointCloud center_cloud;

  sensor_msgs::LaserScan right_scan;

  sensor_msgs::LaserScan left_scan;

  nav_msgs::OccupancyGrid occ_grid;

  nav_msgs::OccupancyGrid inflated_grid;

  const int m_pts_per_ring = 512;

  const int m_num_rings = 16;

  const double center_laser_res = 0.0122718463;

  double m_local_costmap_res;

  double m_local_costmap_height;

  double m_local_costmap_width;

  double m_z_ground_buffer;

  double m_obs_range;

  double m_max_inflation_r;

  int m_grid_array_length;

};

}
