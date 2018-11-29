#include <local_planner.hpp>

namespace Prius {

LocalPlanner::LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/local_costmap", 10, &LocalPlanner::costmapCallback, this);
    local_nav_sub = nh.subscribe<prius_msgs::LocalNav>("/local_nav_waypoints", 100, &LocalPlanner::localNavCallback, this);
    gazebo_state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, &LocalPlanner::gazeboStatesCallback, this);
    mp_pub = nh.advertise<prius_msgs::MotionPlanning>("/mp", 10);
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
    initializePlanner();
    ros::Time start_time = ros::Time::now();
    ros::Duration duration;
    int count = 0;
    while(true)
    {
        duration = ros::Time::now() - start_time;
        if(duration.toSec() > 1)
        {
            ROS_ERROR_STREAM("path plan time exceeded, attempting to replan");
            return;
        }
        GraphNode current_node = m_frontier.top();
        closeNode();
        auto result = checkForGoal(current_node);
        if(result.first)
        {
            openNode(result.second);
            //calcOccGrid();
            calcPathMsg(result.second.child_point);
            calcMPMessage();
            //ROS_INFO_STREAM("path found!");
            return;
        }
        std::vector<GraphNode> neighbors = getNeighbors(current_node);
        for(auto &&neighbor : neighbors)
        {
            markVisited(neighbor.child_point);
            std::vector<GraphNode>::iterator open_it = std::find(m_tree_open.begin(), m_tree_open.end(), neighbor);
            std::vector<GraphNode>::iterator closed_it = std::find(m_tree_closed.begin(), m_tree_closed.end(), neighbor);
            if(closed_it == m_tree_closed.end())
            {
                if(open_it == m_tree_open.end())
                {
                    openNode(neighbor);
                }
                else
                {
                    if(neighbor.g < (*open_it).g)
                    {
                        (*open_it) = neighbor;
                    }
                    openNode(neighbor);
                }
            }
        }
        count++;
        if(count == 10)
        {
            //calcOccGrid();
            count = 0;
        }
        closeNode();
    }
}

void LocalPlanner::initializePlanner()
{
    clearVisited();
    clearTree();
    markGoalPoint();
    publishGoal();
    point current_point = std::make_pair(0, 0);
    double heading = 0;
    double velocity = sqrt(pow(prius_velocity.linear.x, 2) + pow(prius_velocity.linear.y, 2) + pow(prius_velocity.linear.z, 2));
    GraphNode current_node(current_point, current_point, heading, velocity);
    current_node.cost = 9999999999;
    openNode(current_node);
}

void LocalPlanner::openNode(const GraphNode &node)
{
    m_frontier.push(node);
    m_tree_open.push_back(node);
}

void LocalPlanner::closeNode()
{
    std::vector<GraphNode>::iterator it = std::find(m_tree_open.begin(), m_tree_open.end(), m_frontier.top());
    m_tree_closed.push_back(*it);
    m_tree_open.erase(it);
    m_frontier.pop();
}

double LocalPlanner::calcH(const GraphNode &node)
{
   double speed = node.velocity;
   if(speed == 0)
   {
       speed = 0.01;
   }
   double goal_x = local_nav.x;
   double goal_y = local_nav.y;
   double max_dist = sqrt(pow(goal_x, 2) + pow(goal_y, 2));
   double dist_to_goal = sqrt(pow(node.child_point.first - std::get<0>(m_goal_pt), 2) + pow(node.child_point.second - std::get<1>(m_goal_pt), 2));
   double dist_heuristic = dist_to_goal / max_dist * 100;
   double yaw_to_goal = atan2(goal_y - node.child_point.second, goal_x - node.child_point.first);
   double yaw_heuristic = abs(node.heading - yaw_to_goal) / (2 * M_PI) * 100;
   double costmap_value = abs(getCSpaceValue(node.child_point)) / 25;
   double speed_heuristic = 100 - speed / m_max_velocity * 100;
   if(checkGoalDist(node))
   {
       speed_heuristic = fabs(speed - local_nav.speed) * 100;
   }
   double cost = fabs(dist_heuristic + yaw_heuristic + speed_heuristic);
   return cost;
}

double LocalPlanner::calcW(const GraphNode &node)
{
    double dx = node.child_point.first - node.parent_point.first;
    double dy = node.child_point.second - node.parent_point.second;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

std::vector<GraphNode> LocalPlanner::getNeighbors(const GraphNode &node)
{
    std::vector<double> possible_velocities = calcPossibleVelocities(node);
    std::vector<double> possible_yaws = calcPossibleYaws(node);
    std::vector<GraphNode> new_nodes;
    for(auto velocity : possible_velocities)
    {
        for(auto yaw : possible_yaws)
        {
            double average_velocity = (node.velocity + velocity);
            double x = node.child_point.first + (average_velocity * m_time_step_ms / 1000) * cos(yaw);
            double y = node.child_point.second + (average_velocity * m_time_step_ms / 1000) * sin(yaw);
            point new_pt(x, y);
            if(!checkCollision(new_pt, yaw))
            {
                GraphNode new_node(new_pt, node.child_point, yaw, velocity);
                new_node.g += calcW(new_node);
                new_node.cost = new_node.g + calcH(new_node);
                new_nodes.push_back(new_node);
            }
        }
    }
    return new_nodes;
}

bool LocalPlanner::checkGoalDist(const GraphNode &node)
{
    double next_vel = node.velocity + local_nav.max_accel * m_time_step_ms / 1000;
    if(next_vel > local_nav.max_speed)
    {
        next_vel = local_nav.max_speed;
    }
    double average_vel = (next_vel + node.velocity) / 2;
    double x = node.child_point.first + average_vel * m_time_step_ms / 1000 * cos(node.heading);
    double y = node.child_point.second + average_vel * m_time_step_ms / 1000 * sin(node.heading);
    double dx = x - local_nav.x;
    double dy = y - local_nav.y;
    double dist_to_goal = sqrt(pow(dx, 2) + pow(dy, 2));
    double speed = node.velocity;
    double max_accel = local_nav.max_accel;
    double time_to_decel = (speed - local_nav.speed) / max_accel;
    double dist_to_decel = speed * time_to_decel - local_nav.max_accel * pow(time_to_decel, 2) / 2;
    return (dist_to_goal + m_goal_pos_tolerance <= dist_to_decel);
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
        if(velocity == 0 && local_nav.speed != 0)
        {
            continue;
        }
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
    return m_c_space_visited[pt.first][pt.second] == 1;
}

void LocalPlanner::clearTree()
{
    m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>, CheaperCost>();
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

std::pair<bool, GraphNode> LocalPlanner::checkForGoal(const GraphNode &node)
{
    auto points = interpolatePoints(node.child_point, node.parent_point);
    double speed = node.velocity;
    double goal_x = std::get<0>(m_goal_pt);
    double goal_y = std::get<1>(m_goal_pt);
    double goal_speed = std::get<2>(m_goal_pt);
    for(auto pt : points)
    {
        double x = pt.first;
        double y = pt.second;
        if(abs(x - goal_x) <= m_goal_pos_tolerance && abs(y - goal_y) <= m_goal_pos_tolerance && abs(speed - goal_speed) <= m_goal_speed_tolerance)
        {            
            return std::make_pair(true, interpolateToGoalPoint(node, point(x, y)));
        }
    }
    GraphNode dummy_node(node.child_point, node.parent_point, 0, 0);
    return std::make_pair(false, dummy_node);
}

GraphNode LocalPlanner::interpolateToGoalPoint(const GraphNode &node, const point &goal_pt)
{
    double x = goal_pt.first;
    double y = goal_pt.second;
    double dx = x - node.child_point.first;
    double dy = y - node.child_point.second;
    double angle = atan2(dy, dx);
    double end_velocity = local_nav.speed;
    return GraphNode(point(x, y), node.child_point, angle, end_velocity);

}

std::vector<point> LocalPlanner::interpolatePoints(point start, const point &end)
{
    std::vector<point> pts_bet;
    double d_x = start.first - end.first;
    double d_y = start.second - end.second;
    double angle_bet = atan2(d_y, d_x);
    double dist_bet = sqrt(pow(d_x, 2) + pow(d_y, 2));
    int num_pts_bet = dist_bet / m_search_res * 2;
    for(int i = 0; i < num_pts_bet; i++)
    {
        double x = start.first + i / double(num_pts_bet) * dist_bet * cos(angle_bet);
        double y = start.second + i / double(num_pts_bet) * dist_bet * sin(angle_bet);
        pts_bet.push_back(point(x, y));
    }
    return pts_bet;
}

void LocalPlanner::markVisited(const point &pt)
{
    int x = m_local_costmap_width / 2 - pt.second / m_local_costmap_res;
    int y = m_local_costmap_height / 2 - pt.first / m_local_costmap_res;
    if(m_c_space_visited[x][y] != 100)
    {
        m_c_space_visited[x][y] = 1;
    }
}

void LocalPlanner::setupCollision(ros::NodeHandle &pnh)
{
    prius_model.initParam("robot_description");
    auto length_result = calcCollisionDistance("front_right_middle_sonar_link", "back_right_middle_sonar_link");
    m_car_length = length_result.first;
    auto width_result = calcCollisionDistance("rear_right_wheel", "rear_left_wheel");
    m_car_width = width_result.first;
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

point LocalPlanner::calcCollisionDistance(const std::string &link_1, const std::string &link_2)
{
    tf::StampedTransform transform = getTransform(link_1, link_2);
    double distance_x = abs(transform.getOrigin().getX());
    double distance_y = abs(transform.getOrigin().getY());
    return point(distance_x, distance_y);
}

tf::StampedTransform LocalPlanner::getTransform(const std::string &link_1, const std::string &link_2)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while(!listener.canTransform(link_1, "base_link", ros::Time(0)) && !listener.canTransform(link_2, "base_link", ros::Time(0)))
    {
        ros::Duration d(0.1);
        d.sleep();
    }
    try
    {
        listener.lookupTransform(link_1, link_2, ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    return transform;
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
    pnh.getParam("collision_buffer_distance", m_collision_buffer_distance);
    pnh.getParam("time_step_ms", m_time_step_ms);
    pnh.getParam("velocity_res", m_velocity_res);
    pnh.getParam("heading_res", m_heading_res);
    pnh.getParam("heading_diff", m_heading_diff);
    pnh.getParam("goal_pos_tolerance", m_goal_pos_tolerance);
    pnh.getParam("goal_speed_tolerance", m_goal_speed_tolerance);
    pnh.getParam("search_res", m_search_res);
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
    return false;
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
            int collision_x = (new_pt.first + x / m_local_costmap_res) * cos(point_angle);
            int collision_y = (new_pt.second + y / m_local_costmap_res) * sin(point_angle);
            point collision_pt = std::make_pair(collision_x, collision_y);
            m_collision_matrix[x].push_back(collision_pt);
            markVisited(point(x, y));
        }
    }
}

void LocalPlanner::publishOccGrid(const nav_msgs::OccupancyGrid &grid)
{
    occ_grid_pub.publish(grid);
}

void LocalPlanner::calcPathMsg(const point &goal_pt)
{
    std::vector<point> reverse_path;
    path.poses.clear();
    point current_point = goal_pt;
    reverse_path.push_back(current_point);
    while(current_point != point(0, 0))
    {
        for(auto node : m_tree_open)
        {
            if(current_point == node.child_point)
            {
                current_point = node.parent_point;
                reverse_path.push_back(current_point);
            }
        }
        for(auto node : m_tree_closed)
        {
            if(current_point == node.child_point)
            {
                current_point = node.parent_point;
                reverse_path.push_back(current_point);
            }
        }
    }
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

void LocalPlanner::calcMPMessage()
{
    prius_msgs::MotionPlanning mp_out;
    mp_out.header.stamp = ros::Time::now();
    mp_out.max_accel = local_nav.max_accel;
    mp_out.max_speed = local_nav.max_speed;
    for(int i = 0; i < path.poses.size() - 2; i++)
    {
        point current_pt(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
        point next_pt(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y);
        GraphNode current_node(current_pt, next_pt, 0, 0);
        GraphNode next_node(current_pt, next_pt, 0, 0);
        for(auto node : m_tree_open)
        {
            if(current_pt == node.child_point)
            {
                current_node = node;
            }
            else if(next_pt == node.child_point)
            {
                next_node = node;
            }
        }
        for(auto node : m_tree_closed)
        {
            if(current_pt == node.child_point)
            {
                current_node = node;
            }
            else if(next_pt == node.child_point)
            {
                next_node = node;
            }
        }
        double d_yaw = next_node.heading - current_node.heading;
        double yaw_rate = d_yaw / (m_time_step_ms / 1000);
        mp_out.durations.push_back(m_time_step_ms);
        mp_out.speeds.push_back(next_node.velocity);
        mp_out.yaw_rates.push_back(yaw_rate);
    }
    publishMPOutput(mp_out);
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
}

void LocalPlanner::localNavCallback(const prius_msgs::LocalNav::ConstPtr &msg)
{
    local_nav = *msg;
    convertGoalToLocalFrame();
    planPath();
}

void LocalPlanner::convertGoalToLocalFrame()
{
    tf::StampedTransform transform;
    tf::TransformListener listener;
    while(!listener.canTransform("/base_link", "/map", ros::Time(0)))
    {
        continue;
    }
    try
    {
        listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
    }
    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    tf::Point global_point(local_nav.x, local_nav.y, 0);
    tf::Point local_point = transform * global_point;
    local_nav.x = local_point.x();
    local_nav.y = local_point.y();
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
