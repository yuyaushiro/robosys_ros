#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "robosys_ros/ball_detector.h"


namespace robosys_ros{

class BallDetectorNodelet : public nodelet::Nodelet
{
 public:
  BallDetectorNodelet(){}
 private:
  void onInit()
  {
    detector_.reset(new BallDetector(getNodeHandle()));
  }

 public:
 private:
  boost::shared_ptr<BallDetector> detector_;
};

}

PLUGINLIB_EXPORT_CLASS(robosys_ros::BallDetectorNodelet, nodelet::Nodelet)
