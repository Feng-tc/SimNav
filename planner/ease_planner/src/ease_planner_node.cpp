#include <ros/ros.h>

#include "ease_planner/ease_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ease_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ease_planner_ns::EasePlanner planner(nh, private_nh);
  planner.initialize();
  ros::spin();
  return 0;
}
