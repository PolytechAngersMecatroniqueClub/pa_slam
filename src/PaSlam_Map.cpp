#include "PaSlam.h"


// those constants should be given by the ros parameter server
// this is something to do
const float cst_resolution = 0.02;     // m/cells
const int cst_height = (int)(15/cst_resolution);           // nb cells
const int cst_width = (int)(15/cst_resolution);            // nb cells
const int cst_cost_stamp_radius = 25; // nb cells
const int cst_max_belief = 100;        // max value in the probability map
const int cst_min_belief = 0;          // min value in the probability map
const int cst_add_belief = 25;         // the value we add when detected an obstacle in the probability map
const int cst_rem_belief = 10;         // the value we remove when detected a free zone in the probability map
const int cst_threshold_belief = 50;   // the value over the one we consider the cell as an obstacle, a free zone otherwise


PaSlam_Map::PaSlam_Map(): _costmap(cst_height, cst_width){

    // getting the constants, should be done with the parameter server...
    _height = cst_height;         //nb cells
    _width = cst_width;           //nb cells
    _resolution = cst_resolution; // m/cells
    _maxBelief = cst_max_belief;
    _minBelief = cst_min_belief;
    _addBelief = cst_add_belief;
    _remBelief = cst_rem_belief;
    _thresholdBelief = cst_threshold_belief;

    init_maps(_height, _width, cst_resolution, cst_cost_stamp_radius);
}

void PaSlam_Map::init_maps(int height, int width, float resolution, int stamp_radius){

    _height = height;
    _width = width;
    _resolution = resolution;

    // probability map initialization
    _probmap.header.frame_id = "map_link";
    _probmap.info.height = _height;         // cells Y
    _probmap.info.width = _width;           // cells X
    _probmap.info.resolution = _resolution; // m/cells;
    // The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.(geometry_msgs/Pose)
    _probmap.info.origin.position.x = -(_width*_resolution)/2.0;
    _probmap.info.origin.position.y = -(_height*_resolution)/2.0;
    _probmap.info.origin.position.z = 0;
    // we clean the map if needed
    _probmap.data.clear();
    _probmap.data.resize(_height*_width);
    // the probability cells are initialized with the _thresholdbelief value 
    for(int i=0; i<_height*_width; i++){
        _probmap.data[i] = _thresholdBelief-1;
    }
    // we initialize the stamp of the costmap
    _costmap.init_costmap(height, width, stamp_radius);
}

// function to get the value of a given cell cx, cy
int PaSlam_Map::prob_get_val_at(int cx, int cy){
    // we first check if the cell is inside the map and then return the value
    if(cx >= 0 && cx < _width && cy >= 0 && cy < _height) return _probmap.data[cx+cy*_width];
    else{
        // -1 is returned otherwise
        ROS_ERROR("PaSlam_Map::prob_get_val_at() - %d %d is not a correct cell", cx, cy);
        return -1;
    }
}

// function to add a value to a given cell
void PaSlam_Map::prob_add_val_at(int cx, int cy, int val){
    // first we check if the cells belongs to the map
    if(cx >= 0 && cx < _width && cy >= 0 && cy < _height){
        // we compute the index of the cell according the cell coordinates
        int id = cx+cy*_width;
        // if the cell become a new obtabcle (it was under the thresholdBelief and now goes over), it is added to the cost map
        if(_probmap.data[id] <= _thresholdBelief && _probmap.data[id] + val > _thresholdBelief) cost_add_obs(cx, cy);
        // the cost is increased beyound a max value
        if( _probmap.data[id] + val <= _maxBelief) _probmap.data[id] += val;
        else _probmap.data[id] = _maxBelief;
    }// if the cell does not belong to the map, nothing to do but warn the user
    else ROS_ERROR("PaSlam_Map::prob_add_val_at() - %d (%d) %d (%d) is not a correct cell", cx, _width, cy, _height);
}

// function to add a value to a given cell
void PaSlam_Map::prob_rem_val_at(int cx, int cy, int val){
    // first we check if the cells belongs to the map
    if(cx >= 0 && cx < _width && cy >= 0 && cy < _height){
        // we compute the index of the cell according the cell coordinates
        int id = cx+cy*_width;
        // if the obstacle disapeared (it goes under the threshold), we need to remove it from the costmap
        if((_probmap.data[id] > _thresholdBelief) && ((_probmap.data[id] - val) < _thresholdBelief)) cost_remove_obs(cx, cy);
        // the cost is decreased over a min value
        if( _probmap.data[id] > val) _probmap.data[id] -= val;
        else _probmap.data[id] = 0;
    }// if the cell does not belong to the map, nothing to do but warn the user
    else ROS_ERROR("PaSlam_Map::prob_rem_val_at() - %d %d is not a correct cell", cx, cy);
}

// to update the map when an obstacle is detected (inputs are cells coordinates)
void PaSlam_Map::obstacle_detected(int x_cell_robot, int y_cell_robot, int x_cell_obs, int y_cell_obs){
    // first we check if the cells belongs to the map
    if(x_cell_obs >= 0 && x_cell_obs < _width && y_cell_obs >= 0 && y_cell_obs < _height){
        // we add the obstacle in the probability map
        prob_add_val_at(x_cell_obs, y_cell_obs, _addBelief);
        // from the robot to this obstacle, the belives of a free path is updated
        free_path(x_cell_robot, y_cell_robot, x_cell_obs, y_cell_obs);
    } // if not, nothing to but but warn the user
    else ROS_ERROR("PaSlam_Map::obstacle_detected() - (%d %d) or (%d %d) is not a correct cell", x_cell_robot, y_cell_robot, x_cell_obs, y_cell_obs);
}

// to update the map when an obstacle is detected (inputs are world coordinates in m)
void PaSlam_Map::obstacle_detected(double x_robot, double y_robot, double x_obs, double y_obs){
    // first we convert the world coordinates into cell indexes
    int x_cell_r = get_x_cell_from_world(x_robot);
    int y_cell_r = get_y_cell_from_world(y_robot);
    int x_cell_o = get_x_cell_from_world(x_obs);
    int y_cell_o = get_y_cell_from_world(y_obs);

    // then we check if the obstacle and the robot belongs to the map
    if(x_cell_r >= 0 && y_cell_r >= 0 && x_cell_o >= 0 && y_cell_o >= 0 &&
       x_cell_r < _width && y_cell_r < _height && x_cell_o < _width && y_cell_o  < _height){
        // and we use the obstacle_detected function that needs cell coordinates
        obstacle_detected(x_cell_r, y_cell_r, x_cell_o, y_cell_o);
    } // if not, nothing to but but warn the user
    else ROS_ERROR("PaSlam_Map::obstacle_detected() - (%d %d) or (%d %d) is not a correct cell", x_cell_r, y_cell_r, x_cell_o, y_cell_o);
}

// function to remove an obstacle from the cost map
void PaSlam_Map::cost_remove_obs(int cx, int cy){
    // This needs to be done...
    _costmap.rem_stamp(cx, cy); // check the PaSlam_Costmap for details
}

// function to add an obstacle to the costmap
void PaSlam_Map::cost_add_obs(int cx, int cy){
    _costmap.add_stamp(cx, cy); // check the PaSlam_Costmap for details
}

// this function convert the x world value into x cell index
int PaSlam_Map::get_x_cell_from_world(double x){
    int cx = (int)((x-_probmap.info.origin.position.x)/_probmap.info.resolution);
}

// this function convert the y world value into y cell index
int PaSlam_Map::get_y_cell_from_world(double y){
   int cy = (int)((y-_probmap.info.origin.position.y)/_probmap.info.resolution);
}

// this function allows to free the path between the robot and the detected obstacle
// in other words, set remove the rem_belief value of all the cell between the robot and the obstacle
// this function is a copy of the one written by Vincent Cueille
// and as it works I did not look deeply inside...
void PaSlam_Map::free_path(int x1, int y1, int x2, int y2){

    bool steep = abs(y2-y1) > abs(x2-x1);

    if(steep) {
        std::swap<int>(x1,y1);
        std::swap<int>(x2,y2);
    }

    if(x1 > x2) {
        std::swap<int>(x1,x2);
        std::swap<int>(y1,y2);
    }

    int dx = x2 - x1;
    int dy = abs(y2 - y1);

    double error = dx/2;
    const int ystep = (y1 < y2) ? 1 : -1;

    int y = y1;
    const int max_x = x2;
    int x_tmp=0, y_tmp=0;

    for(int x = x1 ; x < max_x; x++){
        if(steep){
            x_tmp = y;
            y_tmp = x;
        }
        else{
            x_tmp = x;
            y_tmp = y;
        }

        prob_rem_val_at(x_tmp, y_tmp, _remBelief);

        error -= dy;
        if(error < 0){
            y += ystep;
            error += dx;
        }
    }
}
