#include <ros/ros.h>
#include <ros/param.h>

#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>

// to handle the mapping
#include "src/PaSlam.h"

// this is for the non blocking input
#include <signal.h>
#include <termios.h>
#include <stdio.h>

// this is for the file operations
#include <fstream>
#include <string.h>

// -------  global variables for the map building -------
int maxcost2add;       // max cost for a LiDAR scan to be added into the map
int cpt_no_change_val; // number of iteration where the pose does not change
int watchdog;          // maximal number of iterations
int nb_iterations;     // 

int map_publishing_rate;           // number of iteration (processed LiDAR data) before publishing the probability map
int path_publishing_rate;          // number of iteration (processed LiDAR data) before publishing the path

PaSlam* paSlam; // to handle the nelder and mead, and the maps

std::string base_link_name;     // Base link name (frame)
std::string lidar_link_name;    // Lidar link name (frame)
nav_msgs::Path path;            // the path (trajectory) of the robot
geometry_msgs::Pose2D pose;     // the current pose (according to the current LiDAR scan and current map)
geometry_msgs::Pose2D old_pose; // the last pose

bool map_started = false;           // flag to identify the first iteration
bool stop_mapping = false;          // flag to start/stop the mapping process
bool publish_lidar_pose_tf = true;  // to enable/disable the publishing of the tf to display the LiDAR data

ros::Publisher path_pub;    // to publish the path

// function to initialize all the parameters according to default values or parameters defined in the launch file
void init_parameters(ros::NodeHandle& node){

    // ---------- GLOBAL VARIABLES ---------- 

    // max cost for a LiDAR scan to be added into the map
    if(node.getParam("maxcost2add", maxcost2add)){
        ROS_INFO("pa_slam_node::maxcost2add parameter (cost): %d", maxcost2add);
    }else{
        maxcost2add = 3000;
        ROS_WARN("pa_slam_node::Could not get the maxcost2add parameter, default value (cost): %d", maxcost2add);
    }
    // number of iteration where the pose does not change
    if(node.getParam("cpt_no_change_val", cpt_no_change_val)){
        ROS_INFO("pa_slam_node::cpt_no_change_val parameter (nb iterations): %d", cpt_no_change_val);
    }else{
        cpt_no_change_val = 10;
        ROS_WARN("pa_slam_node::Could not get the cpt_no_change_val parameter, default value (nb iterations): %d", cpt_no_change_val);
    }
    // maximal number of iterations
    if(node.getParam("watchdog", watchdog)){
        ROS_INFO("pa_slam_node::watchdog parameter (nb iterations): %d", watchdog);
    }else{
        watchdog = 40;
        ROS_WARN("pa_slam_node::Could not get the watchdog parameter, default value (nb iterations): %d", watchdog);
    }
    // number of iteration (processed LiDAR data) before publishing the probability map
    map_publishing_rate = 100;
    if(node.getParam("map_publishing_rate", map_publishing_rate)){
        ROS_INFO("pa_slam_node::map_publishing_rate parameter (nb iteration): %d", map_publishing_rate);
    }else{
        ROS_WARN("pa_slam_node::Could not get the map_publishing_rate parameter, default value (nb iteration): %d", map_publishing_rate);
    }
    // number of iteration (processed LiDAR data) before publishing the path
    path_publishing_rate = 10;
    if(node.getParam("path_publishing_rate", path_publishing_rate)){
        ROS_INFO("pa_slam_node::path_publishing_rate parameter (nb iteration): %d", path_publishing_rate);
    }else{
        ROS_WARN("pa_slam_node::Could not get the path_publishing_rate parameter, default value (nb iteration): %d", path_publishing_rate);
    }
    // to enable/disable the publishing of the tf to display the LiDAR data
    publish_lidar_pose_tf = true;
    if(node.getParam("publish_lidar_pose_tf", publish_lidar_pose_tf)){
        ROS_INFO("pa_slam_node::publish_lidar_pose_tf parameter (boolean): %d", publish_lidar_pose_tf);
    }else{
        ROS_WARN("pa_slam_node::Could not get the publish_lidar_pose_tf parameter, default value (boolean): %d", publish_lidar_pose_tf);
    }
    // Base Link name (frame)
    base_link_name = "base_link_default";
    if(node.getParam("base_link_name", base_link_name)){
        ROS_INFO("pa_slam_node::base link name: %s", base_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_node::Could not get the base link name parameter, default value: %s", base_link_name.c_str());
    }
    // LiDAR Link name (frame)
    lidar_link_name = "laser_default";
    if(node.getParam("lidar_link_name", lidar_link_name)){
        ROS_INFO("pa_slam_node::lidar link name: %s", lidar_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_node::Could not get the lidar link name parameter, default value: %s", lidar_link_name.c_str());
    }
    // to publish the path of the robot
    std::string path_topic_name = "/paSlam/path";
    if(node.getParam("path_topic_name", path_topic_name)){
        ROS_INFO("pa_slam_node::path topic name: %s", path_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_node::Could not get the path topic name parameter, default value: %s", path_topic_name.c_str());
    }
    path_pub = node.advertise<nav_msgs::Path>(path_topic_name, 1000);

    //  ---------- PA_SLAM VARIABLES  ---------- 

    // the topic name to publish the probability map
    std::string probmap_topic_name = "/paSlam/probmap";
    if(node.getParam("probmap_topic_name", probmap_topic_name)){
        ROS_INFO("pa_slam_node::probmap topic name: %s", probmap_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_node::Could not get the probmap topic name parameter, default value: %s", probmap_topic_name.c_str());
    }
    // the topic name to publish the cost map
    std::string costmap_topic_name = "/paSlam/costmap";
    if(node.getParam("costmap_topic_name", costmap_topic_name)){
        ROS_INFO("pa_slam_node::costmap topic name: %s", costmap_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_node::Could not get the costmap topic name parameter, default value: %s", costmap_topic_name.c_str());
    }

    //  Build SLAM object
    paSlam = new PaSlam(costmap_topic_name, probmap_topic_name);

    // maximal number of iterations (Nelder & Mead)
    int nb_ite_max = 30;            
    if(node.getParam("nb_ite_max", nb_ite_max)){
        ROS_INFO("pa_slam_node::nb_ite_max parameter (nb iterations): %d", nb_ite_max);
    }else{
        ROS_WARN("pa_slam_node::Could not get the nb_ite_max parameter, default value (nb iterations): %d", nb_ite_max);
    }
    paSlam->_nb_ite_max = nb_ite_max;
    // Map cell resolution (meters)
    float resolution = 0.02;
    if(node.getParam("resolution", resolution)){
        ROS_INFO("pa_slam_node::resolution parameter (meters): %2.3f", resolution);
    }else{
        ROS_WARN("pa_slam_node::Could not get the resolution parameter, default value (meters): %2.3f", resolution);
    }
    // Map maximal height (meters)
    float height = 150;
    if(node.getParam("height", height)){
        ROS_INFO("pa_slam_node::height parameter (meters): %2.2f", height);
    }else{
        ROS_WARN("pa_slam_node::Could not get the height parameter, default value (meters): %2.2f", height);
    }
    // Map maximal width (meters)
    float width = 150;
    if(node.getParam("width", width)){
        ROS_INFO("pa_slam_node::width parameter (meters): %2.2f", width);
    }else{
        ROS_WARN("pa_slam_node::Could not get the width parameter, default value (meters): %2.2f", width);
    }
    // Radius of the costmap stamp (number of cells)
    int cost_stamp_radius = 30;
    if(node.getParam("cost_stamp_radius", cost_stamp_radius)){
        ROS_INFO("pa_slam_node::cost_stamp_radius parameter (nb cells): %d", cost_stamp_radius);
    }else{
        ROS_WARN("pa_slam_node::Could not get the cost_stamp_radius parameter, default value (nb cells): %d", cost_stamp_radius);
    }
    // initialization of the map according to the previous parameters
    paSlam->_map.init_maps((int)(height/resolution+resolution), (int)(width/resolution+resolution)+1, resolution, cost_stamp_radius);
    // maximal value of a cell in the probability map
    int max_belief = 100;
    if(node.getParam("max_belief", max_belief)){
        ROS_INFO("pa_slam_node::max_belief parameter (cost): %d", max_belief);
    }else{
        ROS_WARN("pa_slam_node::Could not get the max_belief parameter, default value (cost): %d", max_belief);
    }
    paSlam->_map._maxBelief = max_belief;
    // minimal value of a cell in the probability map
    int min_belief = 0;
    if(node.getParam("min_belief", min_belief)){
        ROS_INFO("pa_slam_node::min_belief parameter (cost): %d", min_belief);
    }else{
        ROS_WARN("pa_slam_node::Could not get the min_belief parameter, default value (cost): %d", min_belief);
    }
    paSlam->_map._minBelief = min_belief;
    // value to add in a cell when adding an obstacle
    int add_belief = 25;
    if(node.getParam("add_belief", add_belief)){
        ROS_INFO("pa_slam_node::add_belief parameter (cost): %d", add_belief);
    }else{
        ROS_WARN("pa_slam_node::Could not get the add_belief parameter, default value (cost): %d", add_belief);
    }
    paSlam->_map._addBelief = add_belief;
    // value to remove in a cell when removing an obstacle
    int rem_belief = 10;
    if(node.getParam("rem_belief", rem_belief)){
        ROS_INFO("pa_slam_node::rem_belief parameter (cost): %d", rem_belief);
    }else{
        ROS_WARN("pa_slam_node::Could not get the rem_belief parameter, default value (cost): %d", rem_belief);
    }
    paSlam->_map._remBelief = rem_belief;
    // Limit value for a cell, under this threshold the cell is free, above, the cell is an obstacle
    int threshold_belief = 50;
    if(node.getParam("threshold_belief", threshold_belief)){
        ROS_INFO("pa_slam_node::threshold_belief parameter (cost): %d", threshold_belief);
    }else{
        ROS_WARN("pa_slam_node::Could not get the threshold_belief parameter, default value (cost): %d", threshold_belief);
    }
    paSlam->_map._thresholdBelief = threshold_belief;
    // Link name of the map (frame)
    std::string map_link_name = base_link_name;
    if(node.getParam("map_link_name", map_link_name)){
        ROS_INFO("pa_slam_node::map link name: %s", map_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_node::Could not get the map link name parameter, default value: %s", map_link_name.c_str());
    }
    paSlam->_map._probmap.header.frame_id = map_link_name.c_str();

    // init the map started flag
    map_started = false;

}

// this is for the non blocking input
// it restore the keyboard configuration at its initial settings
void RestoreKeyboardBlocking(struct termios *initial_settings){
    tcsetattr(0, TCSANOW, initial_settings);
}

// this is for the non blocking input
// set the keyboard configuration to be non blocking
void SetKeyboardNonBlock(struct termios *initial_settings){
    struct termios new_settings;
    tcgetattr(0,initial_settings);
    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

// to transform a pitch/roll/yaw value to a quaternion
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw){
    geometry_msgs::Quaternion q;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x=(t0 * t3 * t4 - t1 * t2 * t5);
    q.y=(t0 * t2 * t5 + t1 * t3 * t4);
    q.z=(t1 * t2 * t4 - t0 * t3 * t5);
    return q;
}

// To save the path of the robot in order to be able to display it
void updatepath(){
    path.header.frame_id = base_link_name.c_str();
    // the 2D pose needs to be converted into a posestamped
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = base_link_name.c_str();
    pose_stamped.pose.position.x = pose.x;
    pose_stamped.pose.position.y = pose.y;
    pose_stamped.pose.orientation = toQuaternion(0.0f,0.0f,pose.theta);
    // adding the new pose to the path
    path.poses.push_back(pose_stamped);
}

// To publish the LiDAR transform based on the pose computed
void publisLiDARPoseTf(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.x, pose.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link_name.c_str(), lidar_link_name.c_str()));
}

// to add a LiDAR scan data into the map (main mapping function)
void addLidarScan(const sensor_msgs::LaserScan::ConstPtr& msg){
    // first it is needed to test if it is the first iteration
    if(! map_started){
        // if it is the first iteration (no map computed so far)
        ROS_INFO("pa_slam_node::Starting mapping!");
        // initialisation of the LiDAR pose
        pose.x = 0;
        pose.y = 0;
        pose.theta = 0;
        paSlam->add_2_map(*msg, pose); // we directly add the first LiDAR data to the map as the origin of the map
        map_started = true; // we update the flag
        nb_iterations = 0;  // initialization of the number of iterations
        paSlam->publish_probability_map();  // we publish the first computed map
         // if enabled, the corresponding tf is published
        if(publish_lidar_pose_tf) publisLiDARPoseTf();
    }else if(!stop_mapping){
        // if it is not the first iteration and the slam is enable
        int watch_dog = 0; // initialiaztion of the watchdog (not to loop too long)
        do{
            int cpt_no_change = 0; // this counts the number of identical pose computed
            while(cpt_no_change < cpt_no_change_val){ // we are waiting for the nelder and mead to return the same value several times
                old_pose = pose; // we save the previous pose
                pose = paSlam->nelder_mead(*msg, pose); // we compute a new pose with nelder and mead optimization
                // we check if this new pose equals the previous one
                if(old_pose.x == pose.x && old_pose.y == pose.y && old_pose.theta == pose.theta){
                    cpt_no_change++;
                }else{
                    cpt_no_change=0;
                }
            }
            watch_dog++; // update the watch dog not to loop too long
        }while(paSlam->get_cost(*msg, pose) > maxcost2add && watch_dog < watchdog); // if we did too many iterations or the new pose has a valid cost

        // if we did not break the loop because of the watchdog
        if(watch_dog < watchdog){
            // we add the new scan to the map
            paSlam->add_2_map(*msg, pose);
            // we update the path
            updatepath();
            // if enabled, the corresponding tf is published
            if(publish_lidar_pose_tf) publisLiDARPoseTf();
        }else{
            // we break the loop because of the watch dog, meaning that the pose/lidar cost is to high...
            // we do not add the LiDAR scan, we stop the map building
            ROS_ERROR("pa_slam_node::scan not added! - current cost: %2.2f", paSlam->get_cost(*msg, pose));
            stop_mapping = true;
        }
        nb_iterations ++;
        if(nb_iterations%map_publishing_rate == 0){
            paSlam->publish_probability_map();
        }
        if(nb_iterations%path_publishing_rate == 0){
            path_pub.publish(path);
        }
    }
}

// main function
int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "pa_slam_node",1);
    ros::NodeHandle node;

    // suscribers to the LiDAR data
    std::string scan_topic_name = "/scan";
    if(node.getParam("scan_topic_name", scan_topic_name)){
        ROS_INFO("pa_slam_node::scan topic name: %s", scan_topic_name.c_str());
    }else{
        ROS_WARN("pa_slam_node::Could not get the scan topic name parameter, default value: %s", scan_topic_name.c_str());
    }
    ros::Subscriber sub_lidar = node.subscribe(scan_topic_name.c_str(), 1000, addLidarScan);

    ros::Rate loop_rate(10); // frequency of the ros loop (in Hz)

    // to override the terminal configuration to be non blocking
    struct termios term_settings;
    SetKeyboardNonBlock(&term_settings);
    ROS_WARN("pa_slam_node::The terminal configuration is overrided - press 'q' to get initial configuration");
    bool conf_overrided = true;

    init_parameters(node); // initialization of the parameters

    std::string path_output_map = "outputs/probability_demo.csv"; // name of the file if the map is saved

    ROS_INFO("pa_slam_node::Node started waiting for LiDAR scan to start mapping");

    // main ros loop
    while(ros::ok())
    {
        char c = getchar();   // call your non-blocking input function
        if(c!=-1){
            switch(c){
                case 'q': // get the terminal back in track, or override the configuration again...
                    if(conf_overrided){
                        conf_overrided = false;
                        RestoreKeyboardBlocking(&term_settings);
                        ROS_WARN("pa_slam_node::The terminal is back to normal - press 'q' to go back into the non blocking configuration");
                    }else{
                        conf_overrided = true;
                        SetKeyboardNonBlock(&term_settings);
                        ROS_WARN("pa_slam_node::The terminal configuration is overrided - press 'q' to get initial configuration");
                    }
                    break;
                case 'p': // publish the path
                    path_pub.publish(path);
                    ROS_INFO("pa_slam_node::Path published");
                    break;
                case 'm': // publish the probability map
                    paSlam->publish_probability_map();
                    ROS_INFO("pa_slam_node::Map published");
                    break;
                case 'c': // publish the cost map
                    paSlam->publish_cost_map();
                    ROS_INFO("pa_slam_node::CostMap published");
                    break;
                case 'o': // to save the probability map into the csv file
                    if(node.getParam("path_output_map", path_output_map)){
                        ROS_INFO("pa_slam_node::path output map: %s", path_output_map.c_str());
                    }else{
                        ROS_WARN("pa_slam_node::Could not get the path output map parameter, default value: %s", path_output_map.c_str());
                    }
                    paSlam->probability_map_to_csv(path_output_map.c_str(), "\t");
                    ROS_INFO("pa_slam_node::Map saved to csv file");
                    break;
                case -1:
                    // no key has been pressed
                    break;
                case 'Q':
                    // To kill the node
                    exit(0);
                    break;
                default:
                    // one not defined key has been pressed
                    if(conf_overrided){
                        ROS_WARN("pa_slam_node::The terminal configuration is overrided - press 'q' to get initial configuration");
                    }
                    ROS_WARN("pa_slam_node::Key pressed (%d) not valid", c);
                    ROS_INFO("pa_slam_node::available commands:\n'q': change keyboard configuration\n'm': publish map\n'c': publish costmap\n'p': publish path\n'o': save probability map to csv file");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

