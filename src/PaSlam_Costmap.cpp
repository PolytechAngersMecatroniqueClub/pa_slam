#include "PaSlam_Costmap.h"
#include <math.h>
#include <stdio.h>
#include <algorithm>

PaSlam_Costmap::PaSlam_Costmap(int height, int width){

    _height = height;       // nb cell Y
    _width = width;         // nb cell X
    _cells.resize(height*width);
    _radius_stamp = -1;     // the stamp is not initialized

}

void PaSlam_Costmap::init_costmap(int height, int width, int radius){
    _height = height;       // nb cell Y
    _width = width;         // nb cell X

    _cells.clear();
    _cells.resize(height*width);

    init_stamp(radius);
}


// function to initialize the stamp:
// roud stamp with precomputed distances to facilitate the adding of an obstacle in the cost map
void PaSlam_Costmap::init_stamp(int radius){
    _maxcost = radius;      // the maximal cost for a cell is the radius of the stamp (otherwise it is infinity)
    _radius_stamp = radius; // save the radius of the stamp for further use
    _x_stamp.clear();       // clean the corresponding x rows quantity
    _stamp.clear();         // clean the stamp if needed
    int number_of_x;        // in order to compute this value only once

    // the value of the stamp are stored from the top to the bottom, and left to right
    // example for a 3 radius stamp
    //
    //          00 
    //    01 02 03 04 05
    //    06 07 08 09 10
    // 11 12 13 14 15 16 17
    //    18 19 20 21 22
    //    23 24 25 26 27
    //          28
    //
    // with the cell 14 being the center of the stamp (with a cost = 0)

    // Firs we initialize the top half of the stamp (00 to 10 in the example)
    for(int y=0; y<radius; y++){
        // the number of x value for an y line is based on the equation of a circle
        number_of_x =  ((int) (sqrt(pow(radius,2)-pow(radius-y,2))))*2+1;
        // we store the number of x for each line in order to ease to loop over the stamp
        _x_stamp.push_back(number_of_x);
        // for each cell of the stamp we compute the distance from the cell and the center, and store that value
        for(int x=0; x<number_of_x; x++){
            _stamp.push_back(sqrt(pow(x-(int)(number_of_x/2),2)+pow(radius-y,2)));
        }
    }

    // then the middle of the stamp (11 to 17 in the example)
    number_of_x = radius*2+1;
    _x_stamp.push_back(radius*2+1);
    for(int x=0; x<number_of_x; x++){
        _stamp.push_back(sqrt(pow(x-(int)(number_of_x/2),2)));
    }
    
    // and finally the  bottom half of the stamp (from 18 to 28 in the example)
    for(int y=radius-1; y>=0; y--){
        number_of_x =  ((int) (sqrt(pow(radius,2)-pow(radius-y,2))))*2+1;
        _x_stamp.push_back(number_of_x);
        for(int x=0; x<number_of_x; x++){
            _stamp.push_back(sqrt(pow(x-(int)(number_of_x/2),2)+pow(radius-y,2)));
        }
    }
    _number_of_y = radius*2+1; // we also store the number of lines (y) for looping over the stamp

    /*
    //if you want to display the stamp you can uncomment this section
    // display the stamp
    int i=0;
    for(int y=0; y<_number_of_y; y++){
        int number_of_x = _x_stamp[y];
        printf("%d: ", number_of_x);
        for(int x=0; x<radius-number_of_x/2+1; x++){
            printf("      ");
        }
        for(int x=0; x<number_of_x; x++){
            printf(" %2.2f ", _stamp[i]);
            i++;
        }printf("\n");
    }*/
}

// this function allows to convert the vector of cells into an occupancygrid (that can be displayed with rviz...)
// this function handles only the data, the other fields must be initialized elsewhere (frame_id, height, width....)
void PaSlam_Costmap::toOccupancyGrid(nav_msgs::OccupancyGrid& grid){
    // first we initialize the data of the grid
    int nbcells = _height*_width;
    grid.data.clear();
    grid.data.resize(nbcells);
    // then we loop over all the cells
    for(int i=0; i<nbcells; i++){
        double value = _cells[i]._cost;
        if(value > _maxcost){ // note that value can be infinity if not initialized yet!
            value = _maxcost; // as infinity can not be displayed, we consider the maxcost instread
        }
        // the idea is to have for each cell of the grid map a value between 0 and 100
        grid.data[i] = (int)(value*100.0/_maxcost);
    }
}

// this function uses the stamp to add an obstacle in the cost map (coordinates cx, cy)
// note that a cost corresponds to the distance to the closest obstacle
void PaSlam_Costmap::add_stamp(int cx, int cy){
    if(_radius_stamp < 0){
        // if the stamp is not initialized
        ROS_ERROR("PaSlam_Costmap::The stamp must be initialized first!");
        return;
    }   
    int i=0;
    int number_of_x;
    int x_cell_value;
    int y_cell_value;
    int id;
    for(int y=0; y<_number_of_y; y++){ // that is why we stored the number of lines (y)
        number_of_x = _x_stamp[y];
        for(int x=0; x<number_of_x; x++){ // that is why for each line (y) we stored the number of rows (x)
            // we get the x and y values (cell index) of the current stamp cell in the cost map
            x_cell_value = cx + (x-(int)(number_of_x/2));
            y_cell_value = cy + (_radius_stamp-y);
            // we check if the cell is in the cost map (some values could be outside, when considering borders for instance)
            if(x_cell_value >= 0 && x_cell_value < _width &&
               y_cell_value >= 0 && y_cell_value < _height){
                // from the x, y coordinates we can find the index of the cell (index in the vector)
                id = x_cell_value+y_cell_value*_width;
                // as we want to store the shortest distance, the cell is updated with the min of the current value
                // and the new one given by the new obstacle
                if(_cells[id]._cost > _stamp[i]){ // if the cost is updated, we also save the coordinates of the obstacles that generate that cost
                    _cells[id]._cost = _stamp[i];
                    _cells[id]._x_obs = x_cell_value;
                    _cells[id]._y_obs = y_cell_value;
                }
            }// if not, nothing to do but going to the next stamp value
            i++; // this is for looping over all the stamp values
        }
    }
}

void PaSlam_Costmap::rem_stamp(int cx, int cy){
    if(_radius_stamp < 0){
        // if the stamp is not initialized
        ROS_ERROR("PaSlam_Costmap::The stamp must be initialized first!");
        return;
    }   
    // to loop over the cells around the obstacles
    int i=0;
    int number_of_x;
    int x_cell_value;
    int y_cell_value;
    int id; 

    // to loop over the cells around the one we want to update
    int i_2=0;
    int number_of_x_2;
    int x_cell_value_2;
    int y_cell_value_2;
    int id_2;

    // first loop over all the cells around the obstacle we want to remove
    for(int y=0; y<_number_of_y; y++){ // that is why we stored the number of lines (y)
        number_of_x = _x_stamp[y];
        for(int x=0; x<number_of_x; x++){ // that is why for each line (y) we stored the number of rows (x)
            // we get the x and y values (cell index) of the current stamp cell in the cost map
            x_cell_value = cx + (x-(int)(number_of_x/2));
            y_cell_value = cy + (_radius_stamp-y);
            // we check if the cell is in the cost map (some values could be outside, when considering borders for instance)
            if(x_cell_value >= 0 && x_cell_value < _width &&
               y_cell_value >= 0 && y_cell_value < _height){
                // from the x, y coordinates we can find the index of the cell (index in the vector)
                id = x_cell_value+y_cell_value*_width;
                // we test if the current cell is not an obstacle and that the score of this cell has been generated by the obstacle we want to remove
                if(_cells[id]._cost != 0 && _cells[id]._x_obs == cx && _cells[id]._y_obs == cy){
                    // this current cell needs to be updated, we initialize its cost to maxcost
                    _cells[id]._cost = _maxcost;
                    // then we have to loop around this current cell using the stamp and see if there is an ostacle to generate a new cost
                    for(int y_2=0; y_2 < _number_of_y; y_2++){ // that is why we stored the number of lines (y)
                        number_of_x_2 = _x_stamp[y_2];
                        for(int x_2=0; x_2 < number_of_x_2; x_2++){ // that is why for each line (y) we stored the number of rows (x)
                            // we get the x and y values (cell index) of the current stamp cell in the cost map
                            x_cell_value_2 = x_cell_value + (x_2-(int)(number_of_x_2/2));
                            y_cell_value_2 = y_cell_value + (_radius_stamp-y_2);
                            // we check if this new cell is in the cost map (some values could be outside, when considering borders for instance)
                            if(x_cell_value_2 >= 0 && x_cell_value_2 < _width &&
                               y_cell_value_2 >= 0 && y_cell_value < _height){
                                // from the x, y coordinates we can find the index of the cell (index in the vector)
                                id_2 = x_cell_value_2 + y_cell_value_2*_width;
                                // we check if this cell is an obstacle and if the cost to go to this obstacle is better than the current one
                                if(_cells[id_2]._cost == 0 && (_stamp[i_2] < _cells[id]._cost)){
                                    _cells[id]._cost = _stamp[i_2];
                                    _cells[id]._x_obs = x_cell_value_2;
                                    _cells[id]._y_obs = y_cell_value_2;
                                }
                            }
                            i_2 ++; // loop over the second stamp values
                        }
                    }
                }
            }// if not, nothing to do but going to the next stamp value
            i++; // this is for looping over all the stamp values
        }
    }
}

// function that return the cost value of a given cell 
double PaSlam_Costmap::get_cost_at(int cx, int cy){
    // first we check if the cell belongs to the cost map
    if(cx >= 0 && cx < _width && cy >= 0 && cy < _height){
        int id = cx+cy*_width;
        // the test is done in order not to return infinity value...
        if(_cells[id]._cost >= _maxcost) return _maxcost;
        else return _cells[id]._cost;
    }
    else{
        // if it does not, we return maxcost
        ROS_ERROR("PaSlam_Costmap::get_cost_at() - %d (%d) %d (%d) is not a correct cell", cx, _width, cy, _height);
        return _maxcost;
    }
}






