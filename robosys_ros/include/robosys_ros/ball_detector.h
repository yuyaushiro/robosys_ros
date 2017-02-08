#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <image_transport/image_transport.h>
#include <robosys_ros/BallDetection.h>
#include <robosys_ros/BallDetectionArray.h>


namespace robosys_ros{

class BallDetector
{
 public:
  BallDetector(ros::NodeHandle &nh);
  ~BallDetector();
 private:
  void imageCb(const  sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  void extractColor(cv::Mat original_image, cv::Mat &red_image, cv::Mat &blue_image, cv::Mat &yellow_image);
  void detectBall(cv::Mat color_image, cv::Mat &original_image, int blue, int green, int red, BallDetectionArray &ball_detection_array);

 public:
 private:
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher red_pub_;
  image_transport::Publisher blue_pub_;
  image_transport::Publisher yellow_pub_;
  ros::Publisher detections_pub_;
  int width_;
  int height_;
};

}
