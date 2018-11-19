#include <local_costmap.hpp>

namespace Prius {

LocalCostmap::LocalCostmap(ros::NodeHandle &nh, ros::NodeHandle pnh)
{  
    center_laser_sub = nh.subscribe<sensor_msgs::PointCloud>("/prius/center_laser/scan", 100, &LocalCostmap::centerLaserCallback, this);
    right_laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/prius/front_right_laser/scan", 100, &LocalCostmap::rightLaserCallback, this);
    left_laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/prius/front_left_laser/scan", 100, &LocalCostmap::leftLaserCallback, this);
    global_costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/global_costmap", 1, &LocalCostmap::globalCostmapCallback, this);
    occ_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 100);
    double costmap_height_meters;
    double costmap_width_meters;
    pnh.getParam("local_costmap_res", m_local_costmap_res);
    pnh.getParam("local_costmap_height", costmap_height_meters);
    pnh.getParam("local_costmap_width", costmap_width_meters);
    pnh.getParam("z_ground_buffer", m_z_ground_buffer);
    pnh.getParam("obstacle_range", m_obs_range);
    pnh.getParam("max_inflation_radius", m_max_inflation_r);
    m_local_costmap_height = costmap_height_meters / m_local_costmap_res;
    m_local_costmap_width = costmap_width_meters / m_local_costmap_res;
    double global_costmap_height, global_costmap_width;
    pnh.getParam("/global_costmap_node/global_costmap_height", global_costmap_height);
    pnh.getParam("/global_costmap_node/global_costmap_width", global_costmap_width);
    m_global_costmap_height = global_costmap_height / m_local_costmap_res;
    m_global_costmap_width = global_costmap_width / m_local_costmap_res;
    setupCostmap();

}

LocalCostmap::~LocalCostmap()
{
    ROS_INFO_STREAM("Local Costmap Destructor Called");
}

void LocalCostmap::setupCostmap()
{
    tf::StampedTransform transform;

    while(!listener.canTransform("/center_laser_link", "/base_link", ros::Time(0)))
    {
        continue;
    }
    try
    {
        listener.lookupTransform("/base_link", "/center_laser_link", ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
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
    m_grid_array_length = int(m_local_costmap_height * m_local_costmap_width);
    inflated_grid.info.height = m_local_costmap_height;
    inflated_grid.info.width = m_local_costmap_width;
    inflated_grid.info.resolution = m_local_costmap_res;
    clearMap();
}


void LocalCostmap::calcOccGrid()
{
    occ_grid.header.stamp = ros::Time::now();
    clearMap();
    mapPlanarScan(right_scan, "front_right_laser_link");
    mapPlanarScan(left_scan, "front_left_laser_link");
    mapCenterScan();
    inflateGrid();
    mapRoadVectors();
}

void LocalCostmap::mapPlanarScan(const sensor_msgs::LaserScan &scan, const std::string &frame)
{
    tf::StampedTransform transform;
    tf::StampedTransform transform_z;

    try
    {
        listener.lookupTransform(frame, "/center_laser_link", ros::Time(0), transform);
    }

    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    try
    {
        listener.lookupTransform("/map", frame, ros::Time(0), transform_z);
    }

    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    for(int i = 0; i < scan.ranges.size(); i++)
    {
        if(!(scan.ranges[i] > scan.range_max))
        {
            double angle = scan.angle_min + i * scan.angle_increment;
            double x, y;
            if(frame == "front_right_laser_link")
            {
                y = -(scan.ranges[i] * cos(angle) * cos(roll) - transform.getOrigin().getX());
                x = scan.ranges[i] * sin(angle) * cos(roll) - transform.getOrigin().getY();
            }

            if(frame == "front_left_laser_link")
            {
                y = scan.ranges[i] * cos(angle) * cos(roll) - transform.getOrigin().getX();
                x = -(scan.ranges[i] * sin(angle) * cos(roll) - transform.getOrigin().getY());
            }
            auto location = calcGridLocation(x, y);
            double max_range =  fabs(transform_z.getOrigin().getZ() / tan(roll));
            double dist_from_laser = scan.ranges[i];
            if(fabs(y) < max_range && dist_from_laser > m_obs_range)
            {
                occ_grid.data[location] = 100;
            }

            else
            {
                if(occ_grid.data[location] != 100)
                {
                    occ_grid.data[location] = 0;
                }
            }
            double angle_to_base = atan2(y - transform.inverse().getOrigin().getY(), x - transform.inverse().getOrigin().getX());
            double dist_to_base = sqrt(pow(x - transform.inverse().getOrigin().getX(), 2) + pow(y - transform.inverse().getOrigin().getY(), 2));
            int num_pts_to_base = dist_to_base / m_local_costmap_res;
            for(int i = 1; i < num_pts_to_base; i++)
            {
                double x_ = x - i * m_local_costmap_res * cos(angle_to_base);
                double y_ = y - i * m_local_costmap_res * sin(angle_to_base);
                auto location = calcGridLocation(x_, y_);
                if(occ_grid.data[location] != 100)
                {
                    occ_grid.data[location] = 0;
                }
            }
        }
    }
}

void LocalCostmap::mapCenterScan()
{
    tf::StampedTransform transform;

    try
    {
        listener.lookupTransform("/center_laser_link", "/map", ros::Time(0), transform);
    }

    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    for(auto pt : center_cloud.points)
    {
        auto location = calcGridLocation(pt.x, pt.y);
        double dist_from_laser = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        if((pt.z > transform.getOrigin().getZ() + m_z_ground_buffer) && dist_from_laser > m_obs_range)
        {
            occ_grid.data[location] = 100;
        }

        else
        {
            if(occ_grid.data[location] != 100)
            {
                occ_grid.data[location] = 0;
            }
        }
        double angle_to_base = atan2(pt.y, pt.x);
        double dist_to_base = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        int num_pts_to_base = dist_to_base / m_local_costmap_res;
        double x, y;
        for(int i = 1; i < num_pts_to_base; i++)
        {
            if(!pt.z > transform.getOrigin().getZ() + m_z_ground_buffer)
            {
                x = pt.x - i * m_local_costmap_res * cos(angle_to_base);
                y = pt.y - i * m_local_costmap_res * sin(angle_to_base);
                auto location = calcGridLocation(x, y);
                if(occ_grid.data[location] != 100)
                {
                    occ_grid.data[location] = 0;
                }
            }
        }
    }
}

void LocalCostmap::mapRoadVectors()
{
    if(global_grid.data.size() < 1)
    {
        return;
    }
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }

    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }    
    tf::Quaternion q;
    q.setW(transform.getOrigin().w());
    q.setX(transform.getOrigin().x());
    q.setY(transform.getOrigin().y());
    q.setZ(transform.getOrigin().z());
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double total_x = m_local_costmap_height * m_local_costmap_res;
    double total_y = m_local_costmap_width * m_local_costmap_res;
    double min_x = -total_x / 2;
    double min_y = -total_y / 2;
    int num_pts_x = m_local_costmap_height * 2;
    int num_pts_y = m_local_costmap_width * 2;

    for(int i = 0; i < num_pts_x; i++)
    {
        for(int j = 0; j < num_pts_y; j++)
        {
            double global_x_unrotated = min_x + i / double(num_pts_x) * total_x;
            double global_y_unrotated = min_y + j / double(num_pts_y) * total_y;
            tf::Point global_point = transform * tf::Point(global_x_unrotated, global_y_unrotated, 0);
            double global_x = global_point.x();
            double global_y = global_point.y();
            int global_location = calcGlobalGridLocation(point(global_x, global_y));
            tf::Point local_point = transform.inverse() * tf::Point(global_x, global_y, 0);
            int local_location = calcGridLocation(local_point.x(), local_point.y());
            if(occ_grid.data[local_location] > global_grid.data[global_location])
            {
                continue;
            }
            occ_grid.data[local_location] = global_grid.data[global_location];
        }
    }
}

int LocalCostmap::calcGridLocation(const double &x, const double &y)
{
    int width_pos = int(x / m_local_costmap_res + m_local_costmap_width / 2);
    int height_pos = int(y / m_local_costmap_res + m_local_costmap_height / 2);
    int location = width_pos + height_pos * m_local_costmap_width - 1;
    return location;
}

int LocalCostmap::calcGlobalGridLocation(const point &pt)
{
    int x = m_global_costmap_height / 2 - pt.first / m_local_costmap_res;
    int y = m_global_costmap_width / 2 - pt.second / m_local_costmap_res;
    int width_pos = m_global_costmap_width - y;
    int height_pos = m_global_costmap_height - x;
    int location = height_pos + width_pos * m_global_costmap_height;
    return location;
}

point LocalCostmap::calcCartesianCoords(const int &location)
{
    std::div_t result = std::div(location, m_local_costmap_height);
    int height_pos = result.quot;
    int width_pos = result.rem;
    double x = (width_pos - m_local_costmap_width / 2) * m_local_costmap_res;
    double y = (height_pos - m_local_costmap_height / 2) * m_local_costmap_res;
    point coords = std::make_pair(x, y);
    return coords;
}

void LocalCostmap::inflateGrid()
{
    int count = 0;
    std::vector<int> occupied_locations = {};
    for(auto pt : occ_grid.data)
    {
        auto coords = calcCartesianCoords(count);
        double x = coords.first;
        double y = coords.second;
        double dist_to_base = sqrt(pow(x, 2) + pow(y, 2));
        double max_dist = sqrt(pow(m_local_costmap_height * m_local_costmap_res, 2) + pow(m_local_costmap_width * m_local_costmap_res, 2));
        double inflation_radius = pow(dist_to_base / max_dist, 2) * m_max_inflation_r;
        double num_pts_in_circle = inflation_radius / m_local_costmap_res;
        double angle_inc = 2 * M_PI / num_pts_in_circle;
        for(int i = 0; i < num_pts_in_circle; i++)
        {
            double angle = i * angle_inc;
            for(int j = 0; j < num_pts_in_circle; j++)
            {
                double x_ = x + j / num_pts_in_circle * inflation_radius * cos(angle);
                double y_ = y + j / num_pts_in_circle * inflation_radius * sin(angle);
                auto location = calcGridLocation(x_, y_);
                if(location >= 0 && location < m_grid_array_length)
                {
                    if(pt == 100)
                    {
                        occupied_locations.push_back(location);
                    }

                    if(pt == 0)
                    {
                        inflated_grid.data[location] = 0;
                    }
                }
            }
        }
        count++;
    }

    for(auto pt : occupied_locations)
    {
        inflated_grid.data[pt] = 100;
    }

    count = 0;
    for(auto pt : inflated_grid.data)
    {
        if(pt == 100)
        {
            occ_grid.data[count] = pt;
        }

        if(pt == 0)
        {
            occ_grid.data[count] = pt;
        }

        count++;
    }
}

void LocalCostmap::clearMap()
{
    occ_grid.data.clear();
    inflated_grid.data.clear();
    for(int i = 0; i < m_grid_array_length; i++)
    {
        occ_grid.data.push_back(-1);
        inflated_grid.data.push_back(-1);
    }
}

void LocalCostmap::pubOccGrid()
{
    calcOccGrid();
    occ_grid_pub.publish(occ_grid);
}

void LocalCostmap::centerLaserCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    center_cloud = *msg;
    pubOccGrid();
}

void LocalCostmap::rightLaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    right_scan = *msg;
}

void LocalCostmap::leftLaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    left_scan = *msg;
}

void LocalCostmap::globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    global_grid = *msg;
}

}
