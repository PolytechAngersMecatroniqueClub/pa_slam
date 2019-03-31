#include <ros/ros.h>
#include <ros/param.h>

#include <vector>
#include <limits>
#include <nav_msgs/OccupancyGrid.h>

struct CostCell{
    // a cell of a cost map contains 
    // a cost (the distance to the closest obstacle)
    // the coordinate of the obstacle that is responsible for the cost
    double _cost;
    int _x_obs; // will be used when removing obstacles function will be done
    int _y_obs;

    // the cost cells are initialized with infinity value
    CostCell(double cost = std::numeric_limits<double>::infinity(), int x_obs = -1, int y_obs = -1){
        _cost = cost;
        _x_obs = x_obs;
        _y_obs = y_obs;
    }
};

class PaSlam_Costmap{
// a cost map class to handle the cost map of the PaSlam
// the cost of a cell correspond to the distance to the closest obstacle from this cell
public:
    PaSlam_Costmap(int height, int width);

    // to init the stamp : precomputed distances to ease the adding of an obstacle
    void init_stamp(int radius);
    // to convert the cost map into a ros occupancygrid
    void toOccupancyGrid(nav_msgs::OccupancyGrid& grid);
    // add an ostacle to the cost map at the cell cx cy
    void add_stamp(int cx, int cy);;
    // remove an ostacle from the cost map at the cell cx cy
    void rem_stamp(int cx, int cy);
    // return the cost of the cell cx cy
    double get_cost_at(int cx, int cy);

    void init_costmap(int height, int width, int radius);

    std::vector<CostCell> _cells; // the vector that store the cells
    int _height;                  //nb cells X
    int _width;                   //nb cells Y
    double _maxcost;              // the maximum cost for a cell

    int _radius_stamp;          // the stamp is round-ish, this is the radius of it
    int _number_of_y;           // number of y lines for the stamp
    std::vector<int> _x_stamp;  // for each y line, the number of x rows (to loop over the round stamp)
    std::vector<double> _stamp; // the precimputed values of the stamp
};
