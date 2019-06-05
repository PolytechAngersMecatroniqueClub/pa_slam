#include "PaSlam.h"

#include <stdlib.h>     // srand, rand
#include <time.h>       // time
#include <fstream>

const int cst_nb_ite = 10;

PaSlam::PaSlam(const std::string& costmap_topic_name, const std::string& probmap_topic_name){
    // initialization of the publishers
    _probmap_pub = _node.advertise<nav_msgs::OccupancyGrid>(probmap_topic_name.c_str(), 1000);
    _costmap_pub = _node.advertise<nav_msgs::OccupancyGrid>(costmap_topic_name.c_str(), 1000);
    // initialisation for the rand
    srand (time(NULL));

    // variables for the nelder and mead
    _dimension = 3; // Dimensions N=3 (x, y, theta)
    _points = std::vector<geometry_msgs::Pose2D>(_dimension+1);
    _costs = std::vector<double>(_dimension+1);

    _nb_ite_max = cst_nb_ite;
}

// function to add a scan to the maps according to the pose
void PaSlam::add_2_map(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose){

    unsigned int i, size = scan.ranges.size();
    double angle = scan.angle_min;
    double obs_x, obs_y;
    std::vector<float>::const_iterator it_msr; //using an iterator for faster loops
    // loop over the measurements
    for(it_msr=scan.ranges.begin(); it_msr!=scan.ranges.end(); ++it_msr){
        if(*it_msr == 0 || *it_msr >= INFINITY || *it_msr < 0.2f || *it_msr >= 100.0f){
            // I do not know why sometimes we go here... maybe it is because of bad scans...
        }
        else{
            // convert the scan into x y coordinates (according to the pose)
            obs_x = (*it_msr) * cos(angle + pose.theta) + pose.x;
            obs_y = (*it_msr) * sin(angle + pose.theta) + pose.y;

            // add the detected obstacle to the map
            // the pose coordinates are also needed to "free" the line between the pose and the obstacle
            _map.obstacle_detected(pose.x, pose.y, obs_x, obs_y);
        }
        // needed to convert a scan into x,y coordinates
        angle += scan.angle_increment;
    }
}

// to publish the probability map (Occupancygrid)
void PaSlam::publish_probability_map(){
    _probmap_pub.publish(_map._probmap);
}

// to publish the cost map as an Occupancygrid
void PaSlam::publish_cost_map(){
    // the cost map needs to be converted into an occupancygrid
    nav_msgs::OccupancyGrid grid;
    _map._costmap.toOccupancyGrid(grid);

    // the cost map occupancygrid needs to have the same parameters as the probability map
    grid.header.frame_id = _map._probmap.header.frame_id;
    grid.info.height = _map._probmap.info.height;         // cells X
    grid.info.width = _map._probmap.info.width;           // cells Y
    grid.info.resolution = _map._probmap.info.resolution; 
    grid.info.origin.position.x = _map._probmap.info.origin.position.x;
    grid.info.origin.position.y = _map._probmap.info.origin.position.y;
    grid.info.origin.position.z = _map._probmap.info.origin.position.z;
    // publish the costmap
    _costmap_pub.publish(grid);
}

// function to compute the cost of a lidar scan according to a pose (based on the costmap)
double PaSlam::get_cost(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose){

    double angle = scan.angle_min;
    double cost = 0; // variable that will store the cost of the scan, initialized to 0

    double wx, wy; // the world coordinates
    int cx, cy; // the cells indexes

    std::vector<float>::const_iterator it_msr; //using an iterator for faster loops
    for(it_msr=scan.ranges.begin(); it_msr!=scan.ranges.end(); ++it_msr){ // for all the measurements in the scan
        if(*it_msr == 0 || *it_msr >= INFINITY || *it_msr < 0.2f || *it_msr >= 10.0f){
            // I do not know why sometimes we go here... maybe it is because of bad scans...
        }
        else{
            // convert the scan measurement into a world x y coordinates according to the pose
            wx = (*it_msr) * cos(angle + pose.theta) + pose.x;
            wy = (*it_msr) * sin(angle + pose.theta) + pose.y;

            // convert the world coordinates into cells indexes in the map
            cx = _map.get_x_cell_from_world(wx);
            cy = _map.get_y_cell_from_world(wy);

            // add the cost of the cell to the total cost
            cost += _map._costmap.get_cost_at(cx, cy);
        }
        // needed to compute the world coordinates of the measurements
        angle += scan.angle_increment;
    }
    return cost;
}

// function to sort a point vector according to a cost vector (used by the nelder mead algorithm)
void PaSlam::sort_points(std::vector<geometry_msgs::Pose2D>& points, std::vector<double>& costs){
    // this function could be optimized...
    double min;
    int id_min;
    // for all the points, we get the one with the min cost and put it at the beginning of the vector
    // and then start again with the second min and so on...
    for(int i=0; i<points.size(); i++){
        min = costs[i];
        id_min = i;
        for(int j=i; j<points.size(); j++){
            if(costs[j] < min){
                min = costs[j];
                id_min = j;
            }
        }
        // we use the std::swap function
        std::swap<geometry_msgs::Pose2D>(points[i], points[id_min]);
        std::swap<double>(costs[i], costs[id_min]);
    }
}

// this function is used to generate a random double value between two bounds
// it is used by the nelder mead algorithm
double fRand(double fMin, double fMax){
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

// nelder mead algorithm based on wikipedia description
geometry_msgs::Pose2D PaSlam::nelder_mead(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose_init){
    // Dimensions N=3 (x, y, theta)
    int N = _dimension;

    double alpha = 1;   //reflexion constant
    double gamma = 2;   //expansion constant
    double rho = 0.5;   //contraction constant
    double sigma = 0.5; //homothetie constant

    double rand_x = 0.5;     // used to create the initial simplex around the pose_initS
    double rand_y = 0.5;     // used to create the initial simplex around the pose_initS
    double rand_theta = 0.1; // used to create the initial simplex around the pose_initS

    // Chose an initial simplex (N+1 points)
    _points[0].x = pose_init.x;
    _points[0].y = pose_init.y;
    _points[0].theta = pose_init.theta;

    _points[1].x = pose_init.x+fRand(0,rand_x)-rand_x/2;
    _points[1].y = pose_init.y+fRand(0,rand_y)-rand_y/2;
    _points[1].theta = pose_init.theta+fRand(0,rand_theta)-rand_theta/2;

    _points[2].x = pose_init.x+fRand(0,rand_x)-rand_x/2;
    _points[2].y = pose_init.y+fRand(0,rand_y)-rand_y/2;
    _points[2].theta = pose_init.theta+fRand(0,rand_theta)-rand_theta/2;

    _points[3].x = pose_init.x+fRand(0,rand_x)-rand_x/2;
    _points[3].y = pose_init.y+fRand(0,rand_y)-rand_y/2;
    _points[3].theta = pose_init.theta+fRand(0,rand_theta)-rand_theta/2;

    //compute the costs of the points of the initial simplex
    _costs[0] = get_cost(scan, _points[0]);
    _costs[1] = get_cost(scan, _points[1]);
    _costs[2] = get_cost(scan, _points[2]);
    _costs[3] = get_cost(scan, _points[3]);

    // variables of the nelder mead
    geometry_msgs::Pose2D x0;
    geometry_msgs::Pose2D xr;
    double costxr;
    geometry_msgs::Pose2D xe;
    double costxe; 
    geometry_msgs::Pose2D xc;
    double costxc;

    bool loop = true; 
    int nb_ite = 0; // the only considered criteria is the number of iteration in this version

    while(loop){
        // sort from the lowest to the biggest
        sort_points(_points, _costs);

        nb_ite ++;
        loop = false;

        // compute x0, the centroid of all points except xN+1
        x0.x = 0; x0.y = 0; x0.theta = 0; // initialization
        for(int i=0; i<N; i++){ // doing the sums
            x0.x += _points[i].x;
            x0.y += _points[i].y;
            x0.theta += _points[i].theta;
        }
        x0.x /= N; x0.y /= N; x0.theta /= N; // doing the means

        // Compute reflected point xr = x0 + alpha( x0 − xN+1)
        xr.x     = x0.x     + alpha*(x0.x     - _points[N].x);
        xr.y     = x0.y     + alpha*(x0.y     - _points[N].y);
        xr.theta = x0.theta + alpha*(x0.theta - _points[N].theta);
        costxr = get_cost(scan, xr); // cost of the reflected point

        // If the reflected point is better than the second worst, but not better than the best
        if(_costs[0] <= costxr && costxr < _costs[N-1]){
            _costs[N] = costxr;
            _points[N] = xr;
            loop = true;
        }else if(costxr < _costs[0]){ //the reflected point is the best point so far
            // compute the expanded point xe = x0 + gamma(xr−x0)
            xe.x     = x0.x     + gamma*(xr.x     - x0.x);
            xe.y     = x0.y     + gamma*(xr.y     - x0.y);
            xe.theta = x0.theta + gamma*(xr.theta - x0.theta);
            costxe = get_cost(scan, xe);

            if(costxe < costxr){ //If the expanded point is better than the reflected point
                _points[N] = xe;
                _costs[N] = costxe;
            }else{
                _points[N] = xr;
                _costs[N] = costxr;
            }
            loop = true;
        }else if(costxr >= _costs[N-1]){ // Here it is certain that f(xr)>=f(xN)
            //Compute contracted point xc = x0 + rho(xN+1−x0)
            xc.x     = x0.x     + rho*(_points[N].x     - x0.x);
            xc.y     = x0.y     + rho*(_points[N].y     - x0.y);
            xc.theta = x0.theta + rho*(_points[N].theta - x0.theta);
            costxc = get_cost(scan, xc);

            if(costxc < _costs[N]){ //If the contracted point is better than the worst point
                _points[N] = xc;
                _costs[N] = costxc;
                loop = true;
            }else{
                //Homothétie de rapport sigma et de centre x1 :
                for(int i=1; i<N+1; i++){
                    _points[i].x     = _points[0].x     + sigma*(_points[i].x     - _points[0].x);
                    _points[i].y     = _points[0].y     + sigma*(_points[i].y     - _points[0].y);
                    _points[i].theta = _points[0].theta + sigma*(_points[i].theta - _points[0].theta);
                }
                loop = true;
            }
        }
        if(nb_ite > _nb_ite_max){
            loop = false;
        }
    }
    return _points[0];
}


void PaSlam::probability_map_to_csv(const std::string& file_name, const std::string& delimiter){
    std::ofstream ifile;  // the file

    ifile.open(file_name, std::ofstream::out);
    ifile << "Probability_map"     << delimiter << std::to_string(ros::Time::now().toSec())      << std::endl;
    ifile << "height_x_(cells)"    << delimiter << std::to_string(_map._probmap.info.height)     << std::endl;
    ifile << "width_y_(cells)"     << delimiter << std::to_string(_map._probmap.info.width)      << std::endl;
    ifile << "cell_resolution_(m)" << delimiter << std::to_string(_map._probmap.info.resolution) << std::endl;

    for (int x = 0; x<_map._probmap.info.height; x++){
        for (int y = 0; y<_map._probmap.info.width; y++){
            ifile <<  std::to_string(_map.prob_get_val_at(x, y)) << delimiter;
        }
        ifile << std::endl;
    }

    ifile.close();
}

