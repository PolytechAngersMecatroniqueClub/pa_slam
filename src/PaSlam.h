#include <ros/ros.h>
#include <ros/param.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <string.h>

#include "PaSlam_Map.h"

class PaSlam{
// this class contains the maps (probability map and cost map) and the nelder mead algorithm
// the nelder mead algorithm allows to evaluate the "best" pose of a lidar scan according to the cost map
public:
    PaSlam(const std::string& costmap_topic_name, const std::string& probmap_topic_name);

    // function that add the laser scan in the map considering that the sensor is at a pose pose
    void add_2_map(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose);

    // to publish the probability map
    void publish_probability_map();
    // to concert the costmap into a grid map and publish it (debugging purpose)
    void publish_cost_map();

    // to publish the probability map
    void probability_map_to_csv(const std::string& file_name, const std::string& delimiter);

    // evaluate the cost of a laser scan according to the pose pose_init (the cost is based on the cost map)
    double get_cost(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose_init);
    // implementation of the wikipedia nelder mead algorithm
    geometry_msgs::Pose2D nelder_mead(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose_init);
    // function used by the nelder mead algorithm to sort the points according to their costs
    void sort_points(std::vector<geometry_msgs::Pose2D>& points, std::vector<double>& costs);

    PaSlam_Map                   _map;    // contains a probability map and a cost map
    sensor_msgs::PointCloud         _pcloud; // use to convert a scan message to a point cloud

    ros::NodeHandle _node;       // needed for the publishers
    ros::Publisher _probmap_pub; // to publish the probability map (occupancy grid)
    ros::Publisher _costmap_pub; // to publish the cost map, only for debugging purpose, it is not optimized!

    // variables for the nelder and mead
    std::vector<geometry_msgs::Pose2D> _points; 
    std::vector<double> _costs;
    int _dimension;
    int _nb_ite_max;

};
