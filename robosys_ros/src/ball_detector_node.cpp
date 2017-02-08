#include <ros/ros.h>
#include "robosys_ros/ball_detector.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_detector_node");
  ros::NodeHandle nh;
  robosys_ros::BallDetector ball_detector(nh);
  ros::spin();
}
