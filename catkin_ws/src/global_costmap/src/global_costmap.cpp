#include <global_costmap.hpp>

namespace Prius {

GlobalCostmap::GlobalCostmap(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::string &world_file)
{
    global_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/global_costmap", 10);
    pnh.getParam("/local_costmap_node/local_costmap_res", m_local_costmap_res);
    pnh.getParam("road_width", m_road_width);
    pnh.getParam("num_lanes", m_num_lanes);
    double global_costmap_height, global_costmap_width;
    pnh.getParam("global_costmap_height", global_costmap_height);
    pnh.getParam("global_costmap_width", global_costmap_width);
    m_global_costmap_height = global_costmap_height / m_local_costmap_res;
    m_global_costmap_width = global_costmap_width / m_local_costmap_res;
    m_world_file = world_file;
    setupRoadMap();
}

GlobalCostmap::~GlobalCostmap()
{
    ROS_INFO_STREAM("Global Costmap Destructor Called");
}

void GlobalCostmap::readEdges()
{
    std::ifstream infile(m_world_file);
    std::string line;
    while(std::getline(infile, line))
    {
        std::vector<std::string> points;
        boost::split(points, line, boost::is_any_of(","));
        std::vector<point> road_points;
        if(points.size() > 0)
        {
            for(auto point_ : points)
            {
                std::vector<std::string> coords;
                boost::split(coords, point_, boost::is_any_of(" "));
                if(coords.size() == 3)
                {
                    float x = boost::lexical_cast<float>(coords[0]);
                    float y = boost::lexical_cast<float>(coords[1]);
                    road_points.push_back(point(x, y));
                }
            }
        }
        m_sections.push_back(road_points);
    }
}

void GlobalCostmap::setupRoadMap()
{
    readEdges();
    constructRoadmapMatrix();
    createOccGrid();
    clearMap();
    generateRoads();
    calcRoadmap();
    pubRoadmap();
}

void GlobalCostmap::createOccGrid()
{
    roadmap.header.frame_id = "map";
    roadmap.info.resolution = m_local_costmap_res;
    roadmap.info.height = m_global_costmap_height;
    roadmap.info.width = m_global_costmap_width;
    roadmap.info.origin.position.x = -m_global_costmap_width / 8;
    roadmap.info.origin.position.y = -m_global_costmap_height / 8;
    roadmap.info.origin.orientation.w = 1;
}

void GlobalCostmap::clearMap()
{
    roadmap.data.clear();
    for(int i = 0; i < (m_global_costmap_height * m_global_costmap_width); i++)
    {
        roadmap.data.push_back(-1);
    }
}

int GlobalCostmap::calcGridLocation(const point &pt)
{
    int width_pos = m_global_costmap_width - pt.second;
    int height_pos = m_global_costmap_height - pt.first;
    int location = height_pos + width_pos * m_global_costmap_height;
    return location;
}

void GlobalCostmap::generateRoads()
{
    for(auto section : m_sections)
    {
        for(int i = 0; i < section.size() -1; i++)
        {
            point start = section[i];
            point end = section[i + 1];
            auto points_bet = interpolatePoints(start, end, true);
            double d_x = start.first - end.first;
            double d_y = start.second - end.second;
            double angle_bet = atan2(d_y, d_x);
            double perp_angle = angle_bet + M_PI / 2;
            double lane_width = m_road_width / m_num_lanes;
            for(auto pt : points_bet)
            {
                std::vector<point> lane_centers;
                double first_lane_x = pt.first - (m_road_width / 2 + lane_width / 2) * cos(perp_angle);
                double first_lane_y = pt.second - (m_road_width / 2 + lane_width / 2)* sin(perp_angle);
                for(int j = 0; j < m_num_lanes; j++)
                {
                    double lane_x = first_lane_x + j * lane_width * cos(perp_angle);
                    double lane_y = first_lane_y + j * lane_width * sin(perp_angle);
                    lane_centers.push_back(point(lane_x, lane_y));
                }
                for(auto lane_center : lane_centers)
                {
                    double end_x_bot = lane_center.first - lane_width / 2 * cos(perp_angle);
                    double end_y_bot = lane_center.second - lane_width / 2 * sin(perp_angle);
                    double end_x_top = lane_center.first + lane_width / 2 * cos(perp_angle);
                    double end_y_top = lane_center.second + lane_width / 2 * sin(perp_angle);
                    point bot_of_lane(end_x_bot, end_y_bot);
                    point top_of_lane(end_x_top, end_y_top);
                    std::vector<point> bot_points = interpolatePoints(lane_center, bot_of_lane, false);;
                    std::vector<point> top_points = interpolatePoints(lane_center, top_of_lane, false);
                    int count = 0;
                    int num_pts = bot_points.size();
                    for(auto pt : bot_points)
                    {
                        double value = 99 - double(count) / double(num_pts) * 99;
                        point matrix_pt = calcRoadmapMatrixCoords(pt);
                        if(matrix_pt.first < 0 || matrix_pt.first > m_roadmap_matrix.size() - 1 || matrix_pt.second < 0 || matrix_pt.second > m_roadmap_matrix[0].size() - 1)
                        {
                            continue;
                        }
                        if(m_roadmap_matrix[matrix_pt.first][matrix_pt.second] > value)
                        {
                            m_roadmap_matrix[matrix_pt.first][matrix_pt.second] = value;
                        }
                        count++;
                    }
                    count = 0;
                    num_pts = top_points.size();
                    for(auto pt : top_points)
                    {
                        double value = 99 - double(count) / double(num_pts) * 99;
                        point matrix_pt = calcRoadmapMatrixCoords(pt);
                        if(matrix_pt.first < 0 || matrix_pt.first > m_roadmap_matrix.size() - 1 || matrix_pt.second < 0 || matrix_pt.second > m_roadmap_matrix[0].size() - 1)
                        {
                            continue;
                        }
                        if(m_roadmap_matrix[matrix_pt.first][matrix_pt.second] > value)
                        {
                            m_roadmap_matrix[matrix_pt.first][matrix_pt.second] = value;
                        }
                        count++;
                    }
                }
            }
        }
    }
}

std::vector<point> GlobalCostmap::interpolatePoints(point start, const point &end, const bool &road)
{
    std::vector<point> pts_bet;
    double d_x = start.first - end.first;
    double d_y = start.second - end.second;
    double angle_bet = atan2(d_y, d_x);
    if(road)
    {
        d_x += m_road_width * cos(angle_bet);
        d_y += m_road_width * sin(angle_bet);
    }
    double dist_bet = sqrt(pow(d_x, 2) + pow(d_y, 2));
    int num_pts_bet = dist_bet / m_local_costmap_res * 2;
    for(int i = 0; i < num_pts_bet; i++)
    {
        double x = start.first + i / double(num_pts_bet) * dist_bet * cos(angle_bet);
        double y = start.second + i / double(num_pts_bet) * dist_bet * sin(angle_bet);
        pts_bet.push_back(point(x, y));
    }
    return pts_bet;
}

point GlobalCostmap::calcRoadmapMatrixCoords(const point &pt)
{
    int x = m_global_costmap_height / 2 - pt.first / m_local_costmap_res;
    int y = m_global_costmap_width / 2 - pt.second / m_local_costmap_res;
    return point(x, y);
}

void GlobalCostmap::constructRoadmapMatrix()
{
    int matrix_height = m_global_costmap_height;
    int matrix_width = m_global_costmap_width;
    for(int i = 0; i < matrix_height; i++)
    {
        m_roadmap_matrix.push_back({});
        for(int j = 0; j < matrix_width; j++)
        {
            m_roadmap_matrix[i].push_back(99);
        }
    }
}

void GlobalCostmap::calcRoadmap()
{
    for(int x = 0; x < m_roadmap_matrix.size(); x++)
    {
        for(int y = 0; y < m_roadmap_matrix[x].size(); y++)
        {
            auto location = calcGridLocation(point(x, y));
            roadmap.data[location] = m_roadmap_matrix[x][y];
        }
    }
}

void GlobalCostmap::pubRoadmap()
{
    global_costmap_pub.publish(roadmap);
}


}
