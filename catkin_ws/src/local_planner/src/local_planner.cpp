#include <local_planner.hpp>

namespace Prius {

LocalPlanner::LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/local_costmap", 10, &LocalPlanner::costmapCallback, this);
    local_nav_sub = nh.subscribe<prius_msgs::LocalNav>("/local_navigation", 100, &LocalPlanner::localNavCallback, this);
    gazebo_state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, &LocalPlanner::gazeboStatesCallback, this);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 100);
    path_pub = nh.advertise<nav_msgs::Path>("/local_path", 10);
    occ_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("planning_scene", 100);
    setupCostmap(pnh);
    setupCSpace();
    setupCollision(pnh);
    getTreeParams(pnh);
}

LocalPlanner::~LocalPlanner()
{
    ROS_INFO_STREAM("LocalPlanner Destructor Called");
}

void LocalPlanner::planPath()
{
    clearVisited();
    clearTree();
    markGoalPoint();
    publishGoal();
    point current_point = std::make_pair(m_car_center_x, m_car_center_y);
    graph_node node = std::make_pair(current_point, current_point);
    m_tree_open.insert(std::make_pair(0, node));
    while(true)
    {
        current_point = m_tree_open.begin()->second.first;        
        if(checkForGoal(current_point))
        {
            calcPathMsg();
            break;
        }       
        std::vector<point> neighbors = getNeighbors(current_point);
        for(auto pt : neighbors)
        {
            markVisited(pt);
            node = std::make_pair(pt, current_point);
            double cost = calculateCost(pt, node);
            bool lower_open_cost = false;
            bool lower_closed_cost = false;
            bool in_open_list = false;
            bool in_closed_list = false;
            for(list::iterator it = m_tree_open.begin(); it != m_tree_open.end(); it++)
            {
                if(pt == it->second.first)
                {
                    in_open_list = true;
                    if(cost <= it->first)
                    {
                        lower_open_cost = true;
                        break;
                    }
                }
                if(lower_open_cost)
                {
                    break;
                }
            }
            if(!in_open_list)
            {
                for(list::iterator it = m_tree_closed.begin(); it != m_tree_closed.end(); it++)
                {
                    if(pt == it->second.first)
                    {
                        in_closed_list = true;
                        if(cost <= it->first)
                        {
                            lower_closed_cost = true;
                            openNode(pt, node);
                            m_tree_closed.erase(it);
                            break;
                        }
                    }
                    if(lower_closed_cost)
                    {
                        break;
                    }
                }
            }
            if(!in_open_list && !in_closed_list)
            {
                openNode(current_point, node);
            }
        }
        closeNode();
    }
}

void LocalPlanner::openNode(const point &current_point, const graph_node &node)
{
    double cost = calculateCost(current_point, node);
    m_tree_open.insert(std::make_pair(cost, node));
}

void LocalPlanner::closeNode()
{
    m_tree_closed.insert(std::make_pair(m_tree_open.begin()->first, m_tree_open.begin()->second));
    m_tree_open.erase(m_tree_open.begin());
}

double LocalPlanner::calculateCost(const point &pt, const graph_node &node)
{
    return m_tree_open.begin()->first + sqrt(pow(pt.first - node.first.first, 2) + pow(pt.second - node.first.second, 2) * m_local_costmap_res);
}

std::vector<point> LocalPlanner::getNeighbors(const point &pt)
{
    std::vector<point> possible_neighbors;
    std::vector<point> neighbors;
    point new_pt = std::make_pair(pt.first + 1, pt.second);
    new_pt = std::make_pair(pt.first - 1, pt.second);
    possible_neighbors.push_back(new_pt);
    new_pt = std::make_pair(pt.first, pt.second + 1);
    possible_neighbors.push_back(new_pt);
    new_pt = std::make_pair(pt.first, pt.second - 1);
    possible_neighbors.push_back(new_pt);
    new_pt = std::make_pair(pt.first + 1, pt.second + 1);
    possible_neighbors.push_back(new_pt);
    new_pt = std::make_pair(pt.first + 1, pt.second - 1);
    possible_neighbors.push_back(new_pt);
    new_pt = std::make_pair(pt.first - 1, pt.second + 1);
    possible_neighbors.push_back(new_pt);
    new_pt = std::make_pair(pt.first - 1, pt.second - 1);
    possible_neighbors.push_back(new_pt);
    for(auto possible_pt : possible_neighbors)
    {
        if(checkBounds(possible_pt) && !checkCollision(possible_pt))
        {
            neighbors.push_back(possible_pt);
        }
    }
    return neighbors;
}



bool LocalPlanner::checkBounds(const point &pt)
{
    if(pt.first > m_local_costmap_width || pt.first < 0 || pt.second > m_local_costmap_height || pt.second < 0)
    {
        return false;
    }
    return true;
}

int LocalPlanner::getCSpaceValue(const point &pt)
{
    return m_c_space[pt.first][pt.second];
}

bool LocalPlanner::checkVisited(const point &pt)
{
    if(m_c_space_visited[pt.first][pt.second] == 1)
    {
        return true;
    }
    return false;
}

void LocalPlanner::clearTree()
{
    m_tree_open.clear();
    m_tree_closed.clear();
}

void LocalPlanner::clearVisited()
{
    m_c_space_visited.clear();
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        m_c_space_visited.push_back({});
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            m_c_space_visited[i].push_back(0);
        }
    }
}

void LocalPlanner::markGoalPoint()
{
    m_goal_pt = std::make_pair(int(m_local_costmap_width - 1 -  m_local_costmap_width / 3), int(m_local_costmap_height - m_local_costmap_height / 3));
    m_c_space_visited[m_goal_pt.first][m_goal_pt.second] = 100;
}


bool LocalPlanner::checkForGoal(const point &pt)
{
    if(m_c_space_visited[pt.first][pt.second] == 100)
    {
        return true;
    }
    return false;
}


void LocalPlanner::markVisited(const point &pt)
{
    if(m_c_space_visited[pt.first][pt.second] != 100)
    {
        m_c_space_visited[pt.first][pt.second] = 1;
    }
}

void LocalPlanner::setupCollision(ros::NodeHandle &pnh)
{
    prius_model.initParam("robot_description");

    m_car_length = calcCollisionDistance("front_right_middle_sonar_link", "back_right_middle_sonar_link");
    m_car_width = calcCollisionDistance("rear_right_wheel", "rear_left_wheel");
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while(!listener.canTransform("rear_right_wheel", "center_laser_link", ros::Time(0)))
    {
        ros::Duration d(0.1);
        d.sleep();
    }

    try
    {
        listener.lookupTransform("rear_right_wheel", "center_laser_link", ros::Time(0), transform);
    }

    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    m_car_center_x = m_c_space.size() / 2 - 1;
    m_car_center_y = m_c_space[0].size() / 2 - 1;
    x_offset_pos = m_car_length - abs(transform.getOrigin().getX());
    x_offset_neg = m_car_length - x_offset_pos;
    m_min_collision_point_x = -m_car_width / 2 / m_local_costmap_res - m_collision_buffer_distance / m_local_costmap_res;
    m_max_collision_point_x = m_car_width / 2 / m_local_costmap_res + m_collision_buffer_distance / m_local_costmap_res;
    m_min_collision_point_y = -m_car_length / 2 / m_local_costmap_res - m_collision_buffer_distance / m_local_costmap_res;
    m_max_collision_point_y = m_car_length / 2 / m_local_costmap_res + m_collision_buffer_distance / m_local_costmap_res;
    m_num_x_points = m_car_width / 2 / m_local_costmap_res - m_collision_buffer_distance / m_local_costmap_res;
    m_num_y_points = m_car_width / 2 / m_local_costmap_res + m_collision_buffer_distance / m_local_costmap_res;
}

double LocalPlanner::calcCollisionDistance(const std::string &link_1, const std::string &link_2)
{
    std::pair<tf::StampedTransform, tf::StampedTransform> transforms = getTransforms(link_1, link_2);
    double distance_1 = abs(transforms.first.getOrigin().getX());
    double distance_2 = abs(transforms.second.getOrigin().getX());
    double distance = distance_1 + distance_2;
    return distance;
}

std::pair<tf::StampedTransform, tf::StampedTransform> LocalPlanner::getTransforms(const std::string &link_1, const std::string &link_2)
{
    tf::TransformListener listener;
    tf::StampedTransform transform_1, transform_2;

    while(!listener.canTransform(link_1, "base_link", ros::Time(0)) && !listener.canTransform(link_2, "base_link", ros::Time(0)))
    {
        ros::Duration d(0.1);
        d.sleep();
    }

    try
    {
        listener.lookupTransform(link_1, "base_link", ros::Time(0), transform_1);
    }

    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    try
    {
        listener.lookupTransform(link_2, "base_link", ros::Time(0), transform_2);
    }

    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    std::pair<tf::StampedTransform, tf::StampedTransform> transforms = std::make_pair(transform_1, transform_2);
    return transforms;
}

void LocalPlanner::setupCostmap(ros::NodeHandle &pnh)
{
    double costmap_height_meters;
    double costmap_width_meters;
    pnh.getParam("/local_costmap_node/local_costmap_res", m_local_costmap_res);
    pnh.getParam("/local_costmap_node/local_costmap_height", costmap_height_meters);
    pnh.getParam("/local_costmap_node/local_costmap_width", costmap_width_meters);
    m_local_costmap_height = costmap_height_meters / m_local_costmap_res;
    m_local_costmap_width = costmap_width_meters / m_local_costmap_res;
}

void LocalPlanner::setupCSpace()
{
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        m_c_space.push_back({});
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            m_c_space[i].push_back(0);
        }
    }
}

void LocalPlanner::getTreeParams(ros::NodeHandle &pnh)
{

    pnh.getParam("search_radius", m_search_radius);
    pnh.getParam("max_yaw_diff", m_max_yaw_diff);
}

void LocalPlanner::mapCostmapToCSpace(const nav_msgs::OccupancyGrid::ConstPtr &local_costmap)
{
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            auto location = calcGridLocation(i, j);
            m_c_space[i][j] = local_costmap->data[location];
        }
    }
}

int LocalPlanner::calcGridLocation(const double &x, const double &y)
{
    int location = m_local_costmap_height * m_local_costmap_width - x * m_local_costmap_height - y - 1;
    return location;
}

bool LocalPlanner::checkCollision(const point &pt)
{
    calcCollisionMatrix(pt);
    for(auto row : m_collision_matrix)
    {
        for(auto pt: row)
        {
            if(m_c_space[pt.first][pt.second] == 100)
            {
                return true;
            }
        }
    }
    return false;
}

void LocalPlanner::calcCollisionMatrix(const point &pt)
{
    m_collision_matrix.clear();
    m_collision_matrix = {};
    double point_angle = 0;
    for(int x = m_min_collision_point_x; x < m_max_collision_point_x ; x++)
    {
        if(x < 0 || x > m_c_space.size() - 1)
        {
            continue;
        }
        m_collision_matrix.push_back({});
        for(int y = m_min_collision_point_y; y < m_max_collision_point_y ; y++)
        {
            if(y < 0 || y > m_c_space[0].size() - 1)
            {
                continue;
            }
            int collision_x = (pt.first + x * m_num_x_points) * cos(point_angle);
            int collision_y = (pt.second + y * m_num_y_points) * sin(point_angle);
            point collision_pt = std::make_pair(collision_x, collision_y);
            m_collision_matrix[x].push_back(collision_pt);
        }
    }
}

void LocalPlanner::publishOccGrid(const nav_msgs::OccupancyGrid &grid)
{
    occ_grid_pub.publish(grid);
}

void LocalPlanner::calcPathMsg()
{
    std::vector<point> reverse_path;
    point current_point = m_goal_pt;
    reverse_path.push_back(current_point);
    while(current_point != std::make_pair(m_car_center_x, m_car_center_y))
    {
        for(list::iterator it = m_tree_open.begin(); it != m_tree_open.end(); it++)
        {
            if(it->second.first == current_point)
            {
                current_point = it->second.second;
                reverse_path.push_back(current_point);
            }
        }
        for(list::iterator it = m_tree_closed.begin(); it != m_tree_closed.end(); it++)
        {
            if(it->second.first == current_point)
            {                
                current_point = it->second.second;
                reverse_path.push_back(current_point);
            }
        }
    }
    nav_msgs::Path path;
    path.header.frame_id = "center_laser_link";
    path.header.stamp = ros::Time::now();
    for(int i = reverse_path.size() - 1; i >= 0; i--)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "center_laser_link";
        pose.pose.position.x = reverse_path[i].second * m_local_costmap_res - m_local_costmap_height * m_local_costmap_res / 2;
        pose.pose.position.y = reverse_path[i].first * m_local_costmap_res - m_local_costmap_width * m_local_costmap_res / 2;
        path.poses.push_back(pose);
    }
    publishPath(path);
}

void LocalPlanner::calcOccGrid()
{
    tf::StampedTransform transform;
    tf::TransformListener listener;
    nav_msgs::OccupancyGrid occ_grid;
    int m_grid_array_length = int(m_local_costmap_height * m_local_costmap_width);
    occ_grid.data.clear();
    for(int i = 0; i < m_grid_array_length; i++)
    {
        occ_grid.data.push_back(-1);
    }

    while(!listener.canTransform("/center_laser_link", "/base_link", ros::Time(0)))
    {
        continue;
    }

    try
    {
        listener.lookupTransform("/base_link", "/center_laser_link", ros::Time(0), transform);
    }

    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    occ_grid.header.frame_id = "base_link";
    occ_grid.info.resolution = m_local_costmap_res;
    occ_grid.info.origin.position.x = -m_local_costmap_width * m_local_costmap_res / 2 + transform.getOrigin().getX();
    occ_grid.info.origin.position.y = -m_local_costmap_height * m_local_costmap_res / 2 + transform.getOrigin().getY();
    occ_grid.info.origin.position.z = 0;
    occ_grid.info.origin.orientation.w = transform.getRotation().getW();
    occ_grid.info.origin.orientation.x = transform.getRotation().getX();
    occ_grid.info.origin.orientation.y = transform.getRotation().getY();
    occ_grid.info.origin.orientation.z = transform.getRotation().getZ();
    occ_grid.info.height = m_local_costmap_height;
    occ_grid.info.width = m_local_costmap_width;

    for(int i = 0; i < m_c_space_visited.size(); i++)
    {
        for(int j = 0; j < m_c_space_visited[0].size(); j++)
        {
            auto location = calcGridLocation(i, j);
            if(m_c_space_visited[i][j] == 1)
            {
                occ_grid.data[location] = 100;
            }
        }
    }
    publishOccGrid(occ_grid);
}

void LocalPlanner::publishGoal()
{
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "center_laser_link";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = (- m_local_costmap_width / 2 + m_goal_pt.second) * m_local_costmap_res;
    goal.pose.position.y = (-m_local_costmap_height / 2 + m_goal_pt.first) * m_local_costmap_res;
    goal_pub.publish(goal);


}


void LocalPlanner::publishPath(const nav_msgs::Path &path)
{
    path_pub.publish(path);
}


void LocalPlanner::calcYawRateDuration()
{
    prius_velocity.linear.x;
}

void LocalPlanner::publishMPOutput(const prius_msgs::MotionPlanning &mp_out)
{
    mp_pub.publish(mp_out);
}

void LocalPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    mapCostmapToCSpace(msg);
    planPath();
}

void LocalPlanner::localNavCallback(const prius_msgs::LocalNav::ConstPtr &msg)
{
    local_nav = *msg;
}

void LocalPlanner::gazeboStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    for(int i = 0; i < msg->name.size(); i++)
    {
        if(msg->name[i] == "prius")
        {
            prius_velocity = msg->twist[i];
            break;
        }
    }
}

}
