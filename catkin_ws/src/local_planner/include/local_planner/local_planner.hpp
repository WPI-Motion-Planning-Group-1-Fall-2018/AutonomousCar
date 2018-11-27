#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <graph_node.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <prius_msgs/LocalNav.h>
#include <prius_msgs/MotionPlanning.h>
#include <queue>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <urdf/model.h>

typedef std::pair<double, double> point;
typedef std::vector<std::vector<int>> graph;

namespace Prius {

class LocalPlanner{

public:
    LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~LocalPlanner();

private:

    //cb
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void localNavCallback(const prius_msgs::LocalNav::ConstPtr &msg);
    void gazeboStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);

    //planning stuffss
    void planPath();
    void initializePlanner();
    bool checkOpen(const point &old_point, const point &current_point, const std::pair<point, point> &node);
    bool checkClosed(const point &current_point,const GraphNode &node);
    void openNode(const GraphNode &node);
    void closeNode();
    GraphNode makeNode(const point &child_pt, const point &parent_point, const double &heading, const double &velocity);
    double calcH(const GraphNode &node);
    double calcG(const GraphNode &node);
    double calcW(const GraphNode &node);
    std::vector<GraphNode> getNeighbors(const GraphNode &node);
    bool checkGoalDist(const GraphNode &node);
    std::vector<double> calcPossibleVelocities(const GraphNode &node);
    std::vector<double> calcPossibleYaws(const GraphNode &node);
    std::vector<point> calcPointsBetween(const point &current_pt, const point &new_pt);
    int getCSpaceValue(const point &pt);
    bool checkVisited(const point &pt);
    void clearTree();
    void clearVisited();
    void markGoalPoint();
    std::pair<bool, GraphNode> checkForGoal(const GraphNode &node);
    GraphNode interpolateToGoalPoint(const GraphNode &node, const point &goal_pt);
    std::vector<point> interpolatePoints(point start, const point &end);
    void markVisited(const point &pt);
    int calcGridLocation(const double &x, const double &y);
    point calcCSpaceCoords(const point &pt);
    bool checkCollision(const point &pt, const double &yaw);

    //settin all the stuffs up rel gud
    void setupCollision(ros::NodeHandle &pnh);
    double calcCollisionDistance(const std::string &link_1, const std::string &link_2);
    std::pair<tf::StampedTransform, tf::StampedTransform> getTransforms(const std::string &link_1, const std::string &link_2);    
    void setupCostmap(ros::NodeHandle &pnh);
    void setupCSpace();
    void getTreeParams(ros::NodeHandle &pnh);
    void mapCostmapToCSpace(const nav_msgs::OccupancyGrid::ConstPtr &local_costmap);
    void calcCollisionMatrix(const point &pt, const double &yaw);

    //outputssssss
    void calcOccGrid();
    void publishGoal();
    void publishOccGrid(const nav_msgs::OccupancyGrid &grid);
    void calcPathMsg(const point &goal_pt);
    void calcMPMessage(const point &goal_pt);
    void publishPath(const nav_msgs::Path &path);
    void constructMPMsg();
    void calcYawRateDuration();
    void publishMPOutput(const prius_msgs::MotionPlanning &mp_out);

    //pubsub
    ros::Subscriber costmap_sub;
    ros::Subscriber local_nav_sub;
    ros::Subscriber gazebo_state_sub;
    ros::Publisher mp_pub;
    ros::Publisher occ_grid_pub;
    ros::Publisher path_pub;
    ros::Publisher goal_pub;

    //rossy messages and stuff
    prius_msgs::LocalNav local_nav;
    urdf::Model prius_model;
    geometry_msgs::Twist prius_velocity;


    //planning members
    struct CheaperCost
    {
        bool operator()(const GraphNode &n1, const GraphNode &n2) const
        {
            return n1.cost > n2.cost;
        }
    };

    std::priority_queue<GraphNode, std::vector<GraphNode>, CheaperCost> m_frontier;
    std::vector<GraphNode> m_tree_open;
    std::vector<GraphNode> m_tree_closed;
    graph m_c_space = {};
    graph m_c_space_visited = {};
    std::vector<std::vector<point>> m_collision_matrix = {};

    //collision stuff
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

    //if u dont know what this is idk
    std::tuple<double, double, double> m_goal_pt;

    //c-space stuff and constricting parameters
    double m_local_costmap_res;
    int m_local_costmap_height;
    int m_local_costmap_width;
    double m_collision_buffer_distance;
    double m_time_step_ms;
    double m_max_velocity = 15;
    double m_velocity_res;
    double m_heading_res;
    double m_heading_diff;
    double m_goal_pos_tolerance;
    double m_goal_speed_tolerance;
    double m_search_res;

};

}
