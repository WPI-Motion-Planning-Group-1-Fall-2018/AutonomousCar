#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <prius_msgs/LocalNav.h>
#include <prius_msgs/MotionPlanning.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <urdf/model.h>

namespace Prius {

class LocalPlanner{

public:
    LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~LocalPlanner();

private:

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void localNavCallback(const prius_msgs::LocalNav::ConstPtr &msg);

    void publishMPOutput();
    void publishPath();

    void planPath();
    void clearVisited();
    void clearTree();
    void markGoalPoint();
    std::pair<int, int> getRandomPoint();
    std::pair<int, int> calcNearestNode(const std::pair<int, int> &pt);
    std::pair<int, int> interpolatePoint(const std::pair<int, int> &rand_pt, const std::pair<int, int> &nearest_node);
    std::vector<std::pair<int, int>> calcPointsBetween(const std::pair<int, int> &pt1, const std::pair<int, int> &pt2);
    bool checkNewPointsForCollision(const std::vector<std::pair<int, int>> &points_between);
    bool checkNewPointsForGoal(const std::vector<std::pair<int, int>> &points_between);
    std::vector<std::pair<int, int>> calcPointsToGoal(const std::vector<std::pair<int, int>> & points_between);
    bool checkForGoal(const std::pair<int, int> &pt);
    void markVisitedPoints(const std::vector<std::pair<int, int>> &points);
    void addPointsToTree(const std::vector<std::pair<int, int>> &points, std::pair<int, int> nearest_node);
    int calcTreeBranch(const std::pair<int, int> &pt);
    void markVisited(const std::pair<int, int> &pt);
    void setupCollision(ros::NodeHandle &pnh);
    double calcCollisionDistance(const std::string &link_1, const std::string &link_2);
    std::pair<tf::StampedTransform, tf::StampedTransform> getTransforms(const std::string &link_1, const std::string &link_2);
    void setupCostmap(ros::NodeHandle &pnh);
    void setupCSpace();
    void getTreeParams(ros::NodeHandle &pnh);
    void mapCostmapToCSpace(const nav_msgs::OccupancyGrid::ConstPtr &local_costmap);
    int calcGridLocation(const double &x, const double &y);
    bool checkCollision(const std::pair<int, int> &pt);
    void calcCollisionMatrix(const std::pair<int, int> &pt);
    double calcCarAngleAtPoint(const std::pair<int, int> &pt);

    ros::Subscriber costmap_sub;
    ros::Subscriber local_nav_sub;
    ros::Publisher mp_pub;
    ros::Publisher path_pub;

    prius_msgs::LocalNav local_nav;
    urdf::Model prius_model;

    std::vector<std::vector<int>> m_c_space = {};
    std::vector<std::vector<int>> m_c_space_visited = {};
    std::vector<std::vector<std::pair<int, int>>> m_collision_matrix = {};
    std::vector<std::vector<std::pair<int, int>>> m_tree = {};


    double m_car_length;
    double m_car_width;
    int m_car_center_x;
    int m_car_center_y;
    double x_offset_pos;
    double x_offset_neg;
    int m_min_collision_point_x;
    int m_max_collision_point_x;
    int m_min_collision_point_y;
    int m_max_collision_point_y;
    int m_num_x_points;
    int m_num_y_points;

    double m_local_costmap_res;
    int m_local_costmap_height;
    int m_local_costmap_width;
    double m_collision_buffer_distance;
    double m_max_branch_length;

};

}
