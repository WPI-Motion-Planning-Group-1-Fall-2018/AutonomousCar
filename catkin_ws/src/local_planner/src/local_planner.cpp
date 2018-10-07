#include <local_planner.hpp>

namespace Prius {

LocalPlanner::LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/local_costmap", 10, &LocalPlanner::costmapCallback, this);
    local_nav_sub = nh.subscribe<prius_msgs::LocalNav>("/local_navigation", 100, &LocalPlanner::localNavCallback, this);
    gazebo_state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, &LocalPlanner::gazeboStatesCallback, this);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 100);
    path_pub = nh.advertise<nav_msgs::Path>("/local_path", 10);
    occ_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("RRT_tree", 100);
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
    int count = 0;
    bool first_it = true;
    while(true)
    {        
        std::pair<int, int> rand_pt = getRandomPoint();
        if(count == 5)
        {
            calcOccGrid();
            count = 0;
            rand_pt = m_goal_pt;
        }
        if(first_it)
        {
            first_it = false;
            rand_pt = m_goal_pt;
        }
        std::pair<int, int> nearest_node = calcNearestNode(rand_pt);
        std::pair<int, int> new_node_pt = interpolatePoint(rand_pt, nearest_node);
        if(m_c_space_visited[new_node_pt.first][new_node_pt.second] != 1)
        {           
            std::vector<std::pair<int, int>> node_pts_between = calcPointsBetween(new_node_pt, nearest_node);
            addNewBranch(node_pts_between, nearest_node);
            if(!checkNewPointsForCollision(node_pts_between))
            {
                if(checkNewPointsForGoal(node_pts_between))
                {                    
                    //std::vector<std::pair<int, int>> pts_to_goal = calcPointsToGoal(node_pts_between);
                    //addYawToTree(pts_to_goal, nearest_node);
                    m_goal_branch = calcTreeBranch(m_goal_pt);
                    ROS_INFO_STREAM("path found");
                    break;
                }                
                markVisitedPoints(node_pts_between);                
            }           
            else
            {
                m_tree.pop_back();
            }
        }
        count++;
    }
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

void LocalPlanner::clearTree()
{
    m_tree.clear();
    m_branch_angles.clear();
    m_tree = {};
    m_branch_angles = {};
    std::pair<int, int> current_point = std::make_pair(m_car_center_x, m_car_center_y);
    std::vector<std::pair<int, int>> first_branch = {};
    first_branch.push_back(current_point);
    m_tree.push_back(first_branch);
    m_branch_angles.push_back({});
    m_branch_angles[0].push_back(0);
}

void LocalPlanner::markGoalPoint()
{
    m_goal_pt = std::make_pair(int(m_local_costmap_width - 1 - m_local_costmap_width / 2), int(m_local_costmap_height - m_local_costmap_height / 3));
    m_c_space_visited[m_goal_pt.first][m_goal_pt.second] = 100;
}

std::pair<int, int> LocalPlanner::getRandomPoint()
{
    int x = rand() % (m_c_space.size() / 2) + m_local_costmap_width / 4 ;
    int y = rand() % (m_c_space[0].size() / 4) + m_local_costmap_height / 2;
    std::pair<int, int> pt = std::make_pair(x, y);
    return pt;
}

std::pair<int, int> LocalPlanner::calcNearestNode(const std::pair<int, int> &pt)
{
    std::pair<int, int> nearest_node;
    double lowest_distance = sqrt(pow(m_local_costmap_width, 2) + pow(m_local_costmap_height, 2));
    for(auto stem : m_tree)
    {
        for(auto branch : stem)
        {
            double distance = sqrt(pow(pt.first - branch.first, 2) + pow(pt.second - branch.second, 2));
            if(distance < lowest_distance)
            {
                lowest_distance = distance;
                nearest_node = std::make_pair(branch.first, branch.second);
            }
        }
    }
    return nearest_node;
}

std::pair<int, int> LocalPlanner::interpolatePoint(const std::pair<int, int> &rand_pt, const std::pair<int, int> &nearest_node)
{
    double d_x = rand_pt.first - nearest_node.first;
    double d_y = rand_pt.second - nearest_node.second;
    double angle_to_center = atan2(d_y, d_x);
    int x = nearest_node.first + m_max_branch_length * cos(angle_to_center);
    int y = nearest_node.second + m_max_branch_length * sin(angle_to_center);
    std::pair<int, int> pt = std::make_pair(x, y);
    return pt;
}

void LocalPlanner::addNewBranch(const std::vector<std::pair<int, int>> &new_points, const std::pair<int, int> &pt)
{
    auto branch = calcTreeBranch(pt);
    std::vector<std::pair<int, int>> new_branch = m_tree[branch];
    for(auto pt : new_points)
    {
        new_branch.push_back(pt);
    }
    m_tree.push_back(new_branch);
}

std::vector<std::pair<int, int>> LocalPlanner::calcPointsBetween(const std::pair<int, int> &pt1, const std::pair<int, int> &pt2)
{
    std::vector<std::pair<int, int>> points_between = {};
    double d_x = pt2.first - pt1.first;
    double d_y = pt2.second - pt1.second;
    double dist_between = sqrt(pow(d_x, 2) + pow(d_y, 2));
    int num_pts_between = dist_between / m_local_costmap_res;
    double angle_between = atan2(d_y, d_x);

    for(int i = 0; i < num_pts_between; i++)
    {
        int x = pt1.first + i / num_pts_between * dist_between * cos(angle_between);
        int y = pt1.second + i / num_pts_between * dist_between * sin(angle_between);
        std::pair<int, int> pt = std::make_pair(x, y);
        points_between.push_back(pt);
    }
    return points_between;
}

bool LocalPlanner::checkNewPointsForCollision(const std::vector<std::pair<int, int>> &points_between)
{
    for(auto pt : points_between)
    {
        if(checkCollision(pt))
        {
            return true;
        }
    }
    return false;
}

bool LocalPlanner::checkNewPointsForGoal(const std::vector<std::pair<int, int>> &points_between)
{
    int count = 1;
    for(auto pt : points_between)
    {
        if(checkForGoal(pt))
        {
            return true;
        }
        count++;
    }
    return false;
}

std::vector<std::pair<int, int>> LocalPlanner::calcPointsToGoal(const std::vector<std::pair<int, int> > &points_between)
{
    std::vector<std::pair<int, int>> pts_to_goal = {};
    for(auto pt : points_between)
    {
        pts_to_goal.push_back(pt);
        if(checkForGoal(pt))
        {
            return pts_to_goal;
        }
    }
}

bool LocalPlanner::checkForGoal(const std::pair<int, int> &pt)
{
    if(m_c_space_visited[pt.first][pt.second] == 100)
    {
        return true;
    }
    return false;
}

void LocalPlanner::markVisitedPoints(const std::vector<std::pair<int, int>> &points)
{
    for(auto pt : points)
    {
        markVisited(pt);
    }
}

void LocalPlanner::addYawToTree(const std::vector<std::pair<int, int>> &points, std::pair<int, int> nearest_node)
{
    int branch = calcTreeBranch(nearest_node);
    std::vector<double> yaw_rates = m_branch_angles[branch];

    for(auto pt : points)
    {   auto angle = calcCarAngleAtPoint(pt);
        yaw_rates.push_back(angle);
    }
    m_branch_angles.push_back(yaw_rates);
}

int LocalPlanner::calcTreeBranch(const std::pair<int, int> &pt)
{
    int count = 0;
    for(auto stem : m_tree)
    {
        for(auto branch : stem)
        {
            if((branch.first == pt.first && branch.second == pt.second)
                || (branch.first == pt.first + 1 && branch.second == pt.second + 1)
                || (branch.first == pt.first && branch.second == pt.second + 1)
                || (branch.first == pt.first + 1 && branch.second == pt.second))
            {
                return count;
            }
        }
        count++;
    }
}

void LocalPlanner::markVisited(const std::pair<int, int> &pt)
{
    m_c_space_visited[pt.first][pt.second] = 1;
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
    double max_branch_length_meters;
    pnh.getParam("max_branch_length", max_branch_length_meters);
    pnh.getParam("max_yaw_diff", m_max_yaw_diff);
    m_max_branch_length = max_branch_length_meters / m_local_costmap_res;
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
    int location = x + y * m_local_costmap_width - 1;
    return location;
}

bool LocalPlanner::checkCollision(const std::pair<int, int> &pt)
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

void LocalPlanner::calcCollisionMatrix(const std::pair<int, int> &pt)
{
    double point_angle = calcCarAngleAtPoint(pt);
    for(int x = m_min_collision_point_x; x < m_max_collision_point_x ; x++)
    {
        m_collision_matrix.push_back({});
        for(int y = m_min_collision_point_y; y < m_max_collision_point_y ; y++)
        {
            int collision_x = (pt.first + x * m_num_x_points) * cos(point_angle);
            int collision_y = (pt.second + y * m_num_y_points) * sin(point_angle);
            std::pair<int, int> collision_pt = std::make_pair(collision_x, collision_y);
            m_collision_matrix[x].push_back(collision_pt);
        }
    }
}

double LocalPlanner::calcCarAngleAtPoint(const std::pair<int, int> &pt)
{
    auto branch = calcTreeBranch(pt);
    double yaw = 0;
    for(int i = 0; i < m_tree[branch].size() - 1; i++)
    {        
        double d_x = m_tree[branch][i + 1].first - m_tree[branch][i].first;
        double d_y = m_tree[branch][i + 1].second - m_tree[branch][i].second;
        double d_angle = atan2(d_y, d_x);
        yaw += d_angle;
    }
    double d_x = pt.first - m_tree[branch].back().first;
    double d_y = pt.second - m_tree[branch].back().second;
    double d_angle = atan2(d_y, d_x);
    yaw += d_angle;
    return yaw;
}

void LocalPlanner::publishOccGrid(const nav_msgs::OccupancyGrid &grid)
{
    occ_grid_pub.publish(grid);
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
            auto location = calcGridLocation(j, i);
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
    goal.pose.position.z = 0;

    goal_pub.publish(goal);


}

void LocalPlanner::calcPathMsg()
{
    ros::Time current_time = ros::Time::now();
    ros::Time prev_time = ros::Time::now();
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "center_laser_link";
    double yaw = 0;
    double prev_yaw = 0;
    std::pair<int, int> prev_pt;
    for(auto pt : m_tree[m_goal_branch])
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "center_laser_link";
        pose.pose.position.x = (- m_local_costmap_width / 2 + pt.second) * m_local_costmap_res;
        pose.pose.position.y = ( - m_local_costmap_height / 2 + pt.first) * m_local_costmap_res;
        pose.pose.position.z = 0;
        double d_yaw = atan2(pt.second - prev_pt.second, pt.first - prev_pt.second);
        yaw += d_yaw;
        tf::Quaternion tf_q = tf::createQuaternionFromYaw(yaw);
        geometry_msgs::Quaternion q;
        q.w = tf_q.getW();
        q.x = tf_q.getX();
        q.y = tf_q.getY();
        q.z = tf_q.getZ();
        pose.pose.orientation = q;
        path.poses.push_back(pose);

        current_time = ros::Time::now();
        ros::Duration dt = current_time - prev_time;
        double yaw_rate = (yaw - prev_yaw) * dt.toSec();
        m_yaw_rates.push_back(yaw_rate);

        prev_time = current_time;
        prev_yaw = yaw;
        prev_pt = pt;

        publishPath(path);
    }
}

void LocalPlanner::publishPath(const nav_msgs::Path &path)
{
    path_pub.publish(path);
}

void LocalPlanner::constructMPMsg()
{
    prius_msgs::MotionPlanning mp_out;
    mp_out.speed = local_nav.speed;
    mp_out.max_accel = local_nav.max_accel;
    mp_out.max_speed = local_nav.max_speed;
    mp_out.yaw_rates = {};
    for(auto rate : m_yaw_rates)
    {
        mp_out.yaw_rates.push_back(rate);
    }
    publishMPOutput(mp_out);

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
    calcPathMsg();
    //constructMPMsg();
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
