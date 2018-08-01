#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle n;
    tf::TransformListener tfl(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap("my_costmap", tfl);
    dwa_local_planner::DWAPlannerROS dp;
    dp.initialize("my_dwa_planner", &tfl, &costmap);
}
