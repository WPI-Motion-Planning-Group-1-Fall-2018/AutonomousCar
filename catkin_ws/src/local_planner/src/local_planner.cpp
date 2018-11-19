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
    point current_point = std::make_pair(0, 0);
    double heading = 0;
    double velocity = sqrt(pow(prius_velocity.linear.x, 2) + pow(prius_velocity.linear.y, 2) + pow(prius_velocity.linear.z, 2));
    GraphNode current_node = makeNode(current_point, current_point, heading, velocity);
    m_tree_open.insert(std::make_pair(calcH(current_point, current_node), current_node));
    ros::Time start_time = ros::Time::now();
    ros::Duration duration;
    int count = 0;
    while(true)
    {
        duration = ros::Time::now() - start_time;
        if(duration.toSec() > 30)
        {
            ROS_ERROR_STREAM("path plan time exceeded, attempting to replan");
            break;
        }
        current_node = m_tree_open.begin()->second;
        current_point = current_node.child_point;
        if(checkForGoal(current_node))
        {
            //ROS_INFO_STREAM("path found!");
            m_goal_pt = std::make_tuple(current_point.first, current_point.second, current_node.velocity);
            calcPathMsg();
            break;
        }
        std::vector<GraphNode> new_nodes = getNeighbors(current_point, current_node);
        for(auto node : new_nodes)
        {
            current_point = node.child_point;
            bool in_open_list = false;
            bool in_closed_list = false;
            double g = calcG(current_node);
            double h = calcH(current_point, current_node);
            double successor_current_cost = g + h;
            for(list::iterator it = m_tree_open.begin(); it != m_tree_open.end(); it++)
            {
                if(checkForEquality(it->second, node))
                {
                    if(g <= successor_current_cost)
                    in_open_list = true;
                    break;
                }
            }
            if(in_open_list)
            {
                continue;
            }
            if(!in_open_list)
            {
                for(list::iterator it = m_tree_closed.begin(); it != m_tree_closed.end(); it++)
                {
                    if(checkForEquality(it->second, node))
                    {                      
                        in_closed_list = true;
                        if(g <= successor_current_cost)
                        {
                            break;
                        }
                        openNode(current_point, node);
                        m_tree_closed.erase(it);
                        break;
                    }
                }
            }
            if(in_closed_list)
            {
                continue;
            }
            if(!in_open_list && !in_closed_list)
            {
                openNode(current_point, node);
            }
            point visited = calcCSpaceCoords(current_point);
            markVisited(visited);
        }
        count++;
        if(count == 10)
        {
            calcOccGrid();
            count = 0;
        }
        closeNode();
    }
}

void LocalPlanner::openNode(const point &current_point, const GraphNode &node)
{
    double cost = calcG(node) + calcH(current_point, node);
    m_tree_open.insert(std::make_pair(cost, node));
}

void LocalPlanner::closeNode()
{
    m_tree_closed.insert(std::make_pair(m_tree_open.begin()->first, m_tree_open.begin()->second));
    m_tree_open.erase(m_tree_open.begin());
}

GraphNode LocalPlanner::makeNode(const point &child_pt, const point &parent_point, const double &heading, const double &velocity)
{
    GraphNode node;
    node.child_point = child_pt;
    node.parent_point = parent_point;
    node.heading = heading;
    node.velocity = velocity;
    return node;
}

bool LocalPlanner::checkForEquality(const GraphNode &map_node, const GraphNode &current_node)
{
    point map_pt = map_node.child_point;
    point current_pt = current_node.child_point;
    if(abs(map_pt.first - current_pt.first) < m_search_res && abs(map_pt.second - current_pt.second) < m_search_res)
    {
        return true;
    }
    return false;
}

double LocalPlanner::calcH(const point &pt, const GraphNode &node)
{
   double speed = node.velocity;
   if(speed == 0)
   {
       speed = 0.01;
   }
   double goal_x = local_nav.x;
   double goal_y = local_nav.y;
   double max_dist = sqrt(pow(goal_x, 2) + pow(goal_y, 2));
   double dist_to_goal = sqrt(pow(pt.first - std::get<0>(m_goal_pt), 2) + pow(pt.second - std::get<1>(m_goal_pt), 2));
   double dist_heuristic = abs(dist_to_goal / max_dist) * 100;
   double yaw_to_goal = atan2(goal_y - pt.second, goal_x - pt.first);
   double yaw_heuristic = abs(node.heading - yaw_to_goal) / (2 * M_PI) * 1;
   double costmap_value = abs(getCSpaceValue(pt));
   double cost = abs((dist_heuristic + yaw_heuristic + costmap_value) / pow(speed, 2));
   return cost;
}

double LocalPlanner::calcG(const GraphNode &node)
{
    auto current_point = node.child_point;
    auto parent_point = node.parent_point;
    double cost = 0;
    int count = 0;
    while(current_point != point(0, 0))
    {
        double dx = current_point.first - parent_point.first;
        double dy = current_point.second - parent_point.second;
        double dist = sqrt(pow(dx, 2) + pow(dy, 2));
        cost += dist;
        count++;
        if(count == 1)
        {
            current_point = parent_point;
        }
        for(list::iterator it = m_tree_open.begin(); it != m_tree_open.end(); it++)
        {
            if(it->second.child_point == current_point)
            {
                current_point = it->second.parent_point;
            }
        }
        for(list::iterator it = m_tree_closed.begin(); it != m_tree_closed.end(); it++)
        {
            if(it->second.child_point == current_point)
            {
                current_point = it->second.parent_point;
            }
        }
    }
    return cost;
}

std::vector<GraphNode> LocalPlanner::getNeighbors(const point &pt, const GraphNode &node)
{
    calcTimeStepMs(node);
    calcVelocityRes(node);
    calcYawRes(node);
    calcHeadingDiff(node);
    calcSearchRes(node);
    std::vector<double> possible_velocities = calcPossibleVelocities(node);
    std::vector<double> possible_yaws = calcPossibleYaws(node);
    std::vector<GraphNode> new_nodes;
    for(auto velocity : possible_velocities)
    {
        for(auto yaw : possible_yaws)
        {
            double x = pt.first + (velocity * m_time_step_ms / 1000) * cos(yaw);
            double y = pt.second + (velocity * m_time_step_ms / 1000) * sin(yaw);
            std::pair<double, double> new_pt = std::make_pair(x, y);
            if(!checkCollision(new_pt, yaw))
            {
                GraphNode new_node = makeNode(new_pt, pt, yaw, velocity);
                new_nodes.push_back(new_node);
            }
        }
    }
    return new_nodes;
}

void LocalPlanner::calcTimeStepMs(const GraphNode &node)
{
    double velocity = node.velocity;
    double velocity_frac = velocity / m_max_velocity;
    double time_step_range = m_max_time_step_ms - m_min_time_step_ms;
    m_time_step_ms = m_max_time_step_ms - velocity_frac * time_step_range;
}

void LocalPlanner::calcVelocityRes(const GraphNode &node)
{
    double velocity = node.velocity;
    double velocity_frac = velocity / m_max_velocity;
    double velocity_step_range = m_max_velocity_res - m_min_velocity_res;
    m_velocity_res = m_min_velocity_res + velocity_frac * velocity_step_range;
}

void LocalPlanner::calcYawRes(const GraphNode &node)
{
    double velocity = node.velocity;
    double velocity_frac = velocity / m_max_velocity;
    double heading_step_range = m_max_heading_res - m_min_heading_res;
    m_heading_res = m_min_heading_res + velocity_frac * heading_step_range;
}

void LocalPlanner::calcHeadingDiff(const GraphNode &node)
{
    double velocity = node.velocity;
    double velocity_frac = velocity / m_max_velocity;
    double heading_diff_range = m_max_heading_diff - m_min_heading_diff;
    m_heading_diff = m_min_heading_diff + velocity_frac * heading_diff_range;
}

void LocalPlanner::calcSearchRes(const GraphNode &node)
{
    double velocity = node.velocity;
    double velocity_frac = velocity / m_max_velocity;
    double search_res_step_range = m_max_search_res - m_min_search_res;
    m_search_res = m_min_search_res + velocity_frac * search_res_step_range;
}

std::vector<double> LocalPlanner::calcPossibleVelocities(const GraphNode &node)
{
    std::vector<double> possible_velocities = {};
    double max_accel = local_nav.max_accel;
    double max_speed = local_nav.max_speed;
    double current_speed = node.velocity;
    double time_step = m_time_step_ms / 1000;    
    double max_possible_speed = current_speed + max_accel * time_step;
    double min_possible_speed = current_speed - max_accel * time_step;
    double possible_speed_range = max_possible_speed - min_possible_speed;
    int num_possible_speeds = possible_speed_range / m_velocity_res;
    for(int i = 0; i < num_possible_speeds; i++)
    {
        double velocity = min_possible_speed + i * m_velocity_res;
        if(velocity > max_speed || velocity < 0)
        {
            continue;
        }
        possible_velocities.push_back(velocity);
    }
    return possible_velocities;
}

std::vector<double> LocalPlanner::calcPossibleYaws(const GraphNode &node)
{
    std::vector<double> possible_yaws = {};
    double current_yaw = node.heading;
    double max_yaw = current_yaw + m_heading_diff;
    double min_yaw = current_yaw - m_heading_diff;
    double yaw_range = max_yaw - min_yaw;
    double num_possible_yaws = yaw_range / m_heading_res;
    for(int i = 0; i < num_possible_yaws; i++)
    {
        double yaw = min_yaw + i * m_heading_res;
        possible_yaws.push_back(yaw);
    }
    return possible_yaws;
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
    m_goal_pt = std::make_tuple(local_nav.x, local_nav.y, local_nav.speed);
}


bool LocalPlanner::checkForGoal(const GraphNode &node)
{
    double x = node.child_point.first;
    double y = node.child_point.second;
    double speed = node.velocity;
    double goal_x = std::get<0>(m_goal_pt);
    double goal_y = std::get<1>(m_goal_pt);
    double goal_speed = std::get<2>(m_goal_pt);
    if(abs(x - goal_x) <= m_goal_pos_tolerance && abs(y - goal_y) <= m_goal_pos_tolerance && abs(speed - goal_speed) <= m_goal_speed_tolerance)
    {
        return true;
    }
    return false;
}


void LocalPlanner::markVisited(const point &pt)
{
    int x = m_local_costmap_width / 2 - pt.second;
    int y = m_local_costmap_height / 2 - pt.first;
    if(m_c_space_visited[x][y] != 100)
    {
        m_c_space_visited[x][y] = 1;
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

    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    try
    {
        listener.lookupTransform(link_2, "base_link", ros::Time(0), transform_2);
    }

    catch(tf::TransformException ex)
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
    pnh.getParam("max_time_step_ms", m_max_time_step_ms);
    pnh.getParam("min_time_step_ms", m_min_time_step_ms);
    pnh.getParam("max_velocity_res", m_max_velocity_res);
    pnh.getParam("min_velocity_res", m_min_velocity_res);
    pnh.getParam("max_heading_res", m_max_heading_res);
    pnh.getParam("min_heading_res", m_min_heading_res);
    pnh.getParam("max_heading_diff", m_max_heading_diff);
    pnh.getParam("min_heading_diff", m_min_heading_diff);
    pnh.getParam("goal_pos_tolerance", m_goal_pos_tolerance);
    pnh.getParam("goal_speed_tolerance", m_goal_speed_tolerance);
    pnh.getParam("max_search_res", m_max_search_res);
    pnh.getParam("min_search_res", m_min_search_res);
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

bool LocalPlanner::checkCollision(const point &pt, const double &yaw)
{
    calcCollisionMatrix(pt, yaw);
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

point LocalPlanner::calcCSpaceCoords(const point &pt)
{
    int x = pt.first / m_local_costmap_res;
    int y = pt.second / m_local_costmap_res;
    return point(x, y);
}

void LocalPlanner::calcCollisionMatrix(const point &pt, const double &yaw)
{
    m_collision_matrix.clear();
    m_collision_matrix = {};
    int x = -m_local_costmap_width / 2 + pt.second / m_local_costmap_res;
    int y = -m_local_costmap_height / 2 + pt.first / m_local_costmap_res;
    std::pair<int, int> new_pt = std::make_pair(x, y);
    double point_angle = yaw;
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
            int collision_x = (new_pt.first + x * m_num_x_points) * cos(point_angle);
            int collision_y = (new_pt.second + y * m_num_y_points) * sin(point_angle);
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
    point current_point = std::make_pair(std::get<0>(m_goal_pt), std::get<1>(m_goal_pt));
    reverse_path.push_back(current_point);
    while(current_point != point(0, 0))
    {
        for(list::iterator it = m_tree_open.begin(); it != m_tree_open.end(); it++)
        {
            if(it->second.child_point == current_point)
            {
                current_point = it->second.parent_point;
                reverse_path.push_back(current_point);
            }
        }
        for(list::iterator it = m_tree_closed.begin(); it != m_tree_closed.end(); it++)
        {
            if(it->second.child_point == current_point)
            {
                current_point = it->second.parent_point;
                reverse_path.push_back(current_point);
            }
        }
    }
    nav_msgs::Path path;
    path.header.frame_id = "base_link";
    path.header.stamp = ros::Time::now();
    for(int i = reverse_path.size() - 1; i >= 0; i--)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "base_link";
        pose.pose.position.x = reverse_path[i].first;
        pose.pose.position.y = reverse_path[i].second;
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
    goal.header.frame_id = "base_link";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = std::get<0>(m_goal_pt);
    goal.pose.position.y = std::get<1>(m_goal_pt);
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
