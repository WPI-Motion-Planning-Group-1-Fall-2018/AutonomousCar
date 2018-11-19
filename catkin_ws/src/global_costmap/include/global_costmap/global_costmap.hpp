#pragma once
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <tf/tf.h>

typedef std::pair<float, float> point;

namespace Prius {

class GlobalCostmap
{
public:

    GlobalCostmap(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::string &world_file);
    ~GlobalCostmap();
    void pubRoadmap();

private:

    void readEdges();
    void setupRoadMap();
    void createOccGrid();
    void clearMap();
    int calcGridLocation(const point &pt);
    void generateRoads();
    std::vector<point> interpolatePoints(point start, const point &end, const bool &road);
    point calcRoadmapMatrixCoords(const point &pt);
    void constructRoadmapMatrix();
    void calcRoadmap();

    ros::Publisher global_costmap_pub;

    std::string m_world_file;
    std::vector<std::vector<point>> m_sections;
    nav_msgs::OccupancyGrid roadmap;
    double m_global_costmap_height;
    double m_global_costmap_width;
    int m_roadmap_width;
    int m_roadmap_height;
    std::vector<std::vector<int>> m_roadmap_matrix;
    double m_local_costmap_res;
    double m_road_width;
    int m_num_lanes;
};

}
