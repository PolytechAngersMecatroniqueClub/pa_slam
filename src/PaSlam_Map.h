#include <ros/ros.h>
#include <ros/param.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include "PaSlam_Costmap.h"

class PaSlam_Map{
// this class allows to handle the maps needed for the slam (probability map and cost map)
public:
    PaSlam_Map();

    PaSlam_Costmap               _costmap; // the cost map
    // the cost map contains for each cell the distance to the closest obstacle
    nav_msgs::OccupancyGrid         _probmap; // the probability map
    // the probability map contains for each cell the probability (integer between 0 and 100)
    // that the cell is an obstacle (100 -> we are shure it is an obstacle, 0-> we are sure it is a free zone)

    int _height;          // nb cells X
    int _width;           // nb cells Y
    float _resolution;    // m/cells (size of a cell in the world)
    int _addBelief;       // the value that should be added to a cell when detected an obstacle
    int _remBelief;       // the value that should be remove to a cell when detected a free zone
    int _thresholdBelief; // probability over the one we consider the cell as an obstacle (it is considered as a free zone if lower)
    int _minBelief;       // minimal value for the probability (0)
    int _maxBelief;       // maximan value for the probability (100)

    // function that return the value of a cell with cx and cy indexes
    int prob_get_val_at(int cx, int cy);
    // function that adds a value val to a cell cx, cy
    void prob_add_val_at(int cx, int cy, int val);
    // function that removes a value val to a cell cx, cy
    void prob_rem_val_at(int cx, int cy, int val);

    // to update the map when an obstacle is detected, the inputs are cell indexes of the robot and the obstacle
    // note that the position of the robot is needed to update the "free" cells (between the robot and the obstacle)
    void obstacle_detected(int x_cell_robot, int y_cell_robot, int x_cell_obs, int y_cell_obs);

    // to update the map when an obstacle is detected, the inputs are world coordinates of the robot and the obstacle
    // note that the position of the robot is needed to update the "free" cells (between the robot and the obstacle)
    void obstacle_detected(double x_world_robot, double y_world_robot, double x_world_obs, double y_world_obs);

    // this function is not implemented yet
    void cost_remove_obs(int cx, int cy);
    // function to add an obstacle in the cost map
    void cost_add_obs(int cx, int cy);

    // function to set the free cells between the robot and the obstacle
    void free_path(int x1, int y1, int x2, int y2);

    // function to convert an x world coordinate into an x index in the map
    int get_x_cell_from_world(double x);

    // function to convert an y world coordinate into an y index in the map
    int get_y_cell_from_world(double y);

    void init_maps(int height, int width, float resolution, int stamp_radius);
};
