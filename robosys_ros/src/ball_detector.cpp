#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <robosys_ros/BallDetection.h>
#include <robosys_ros/BallDetectionArray.h>
#include "robosys_ros/Labeling.h"
#include "robosys_ros/ball_detector.h"
#include "robosys_ros/color_ball.h"

using namespace ros;
using namespace cv;


namespace robosys_ros{

BallDetector::BallDetector(NodeHandle &nh)
: it_(nh)
{
  image_sub_ = it_.subscribeCamera("image_rect_color", 1, &BallDetector::imageCb, this);
  image_pub_ = it_.advertise("ball_detections_image", 1);
  red_pub_ = it_.advertise("red_extract_image", 1);
  blue_pub_ = it_.advertise("blue_extract_image", 1);
  yellow_pub_ = it_.advertise("yellow_extract_image", 1);
  detections_pub_ = nh.advertise<BallDetectionArray>("ball_detections", 1);
}
BallDetector::~BallDetector()
{
  image_sub_.shutdown();
}

void BallDetector::imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  BallDetectionArray ball_detection_array;

  width_ = cam_info->width;
  height_ = cam_info->height;

  Mat original_image = cv_ptr->image;
  Mat red_image = Mat::zeros(Size(width_, height_), CV_8UC1);
  Mat blue_image = Mat::zeros(Size(width_, height_), CV_8UC1);
  Mat yellow_image = Mat::zeros(Size(width_, height_), CV_8UC1);

  extractColor(original_image, red_image, blue_image, yellow_image);
  detectBall(red_image, original_image, 0, 0, 255, ball_detection_array);
  detectBall(blue_image, original_image, 255, 0, 0, ball_detection_array);
  detectBall(yellow_image, original_image, 0, 255, 255, ball_detection_array);

  //resize(original_image, original_image, Size(), 0.7, 0.7, INTER_LINEAR);
  //resize(red_image, red_image, Size(), 0.7, 0.7, INTER_LINEAR);
  //resize(blue_image, blue_image, Size(), 0.7, 0.7, INTER_LINEAR);
  //resize(yellow_image, yellow_image, Size(), 0.7, 0.7, INTER_LINEAR);

  detections_pub_.publish(ball_detection_array);
  image_pub_.publish(cv_ptr->toImageMsg());
  red_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", red_image).toImageMsg());
  blue_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", blue_image).toImageMsg());
  yellow_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", yellow_image).toImageMsg());
}

void BallDetector::extractColor(Mat original_image, Mat &red_image, Mat &blue_image, Mat &yellow_image)
{
  Mat smooth_image, hsv_image;
  int red_lower[3]={175, 150, 135}, red_upper[3]={5, 255, 255};
  int blue_lower[3]={100, 123, 77}, blue_upper[3]={112, 255, 255};
  int yellow_lower[3]={14, 100, 130}, yellow_upper[3]={25, 165, 255};

  medianBlur(original_image, smooth_image, 9);
  cvtColor(smooth_image, hsv_image, CV_BGR2HSV);

  param::get("red_threshold/hue_upper", red_upper[0]);
  param::get("red_threshold/hue_lower", red_lower[0]);
  param::get("red_threshold/saturation_upper", red_upper[1]);
  param::get("red_threshold/saturation_lower", red_lower[1]);
  param::get("red_threshold/value_upper", red_upper[2]);
  param::get("red_threshold/value_lower", red_lower[2]);
  param::get("blue_threshold/hue_upper", blue_upper[0]);
  param::get("blue_threshold/hue_lower", blue_lower[0]);
  param::get("blue_threshold/saturation_upper", blue_upper[1]);
  param::get("blue_threshold/saturation_lower", blue_lower[1]);
  param::get("blue_threshold/value_upper", blue_upper[2]);
  param::get("blue_threshold/value_lower", blue_lower[2]);
  param::get("yellow_threshold/hue_upper", yellow_upper[0]);
  param::get("yellow_threshold/hue_lower", yellow_lower[0]);
  param::get("yellow_threshold/saturation_upper", yellow_upper[1]);
  param::get("yellow_threshold/saturation_lower", yellow_lower[1]);
  param::get("yellow_threshold/value_upper", yellow_upper[2]);
  param::get("yellow_threshold/value_lower", yellow_lower[2]);

  for(int y = 0; y < height_; y++)
  {
    for(int x = 0; x < width_; x++)
    {
      int a = hsv_image.step*y + (x * 3);
      if((hsv_image.data[a] >= red_lower[0] || hsv_image.data[a] <= red_upper[0]) &&
         hsv_image.data[a+1] >= red_lower[1] && hsv_image.data[a+1] <= red_upper[1] &&
         hsv_image.data[a+2] >= red_lower[2] && hsv_image.data[a+2] <= red_upper[2])
      {
        red_image.data[red_image.step*y + x] = 255;
      }
      if(hsv_image.data[a] >= blue_lower[0] && hsv_image.data[a] <= blue_upper[0] &&
         hsv_image.data[a+1] >= blue_lower[1] && hsv_image.data[a+1] <= blue_upper[1] &&
         hsv_image.data[a+2] >= blue_lower[2] && hsv_image.data[a+2] <= blue_upper[2])
      {
        blue_image.data[blue_image.step*y + x] = 255;
      }
      if(hsv_image.data[a] >= yellow_lower[0] && hsv_image.data[a] <= yellow_upper[0] &&
         hsv_image.data[a+1] >= yellow_lower[1] && hsv_image.data[a+1] <= yellow_upper[1] &&
         hsv_image.data[a+2] >= yellow_lower[2] && hsv_image.data[a+2] <= yellow_upper[2])
      {
        yellow_image.data[yellow_image.step*y + x] = 255;
      }
    }
  }

}

void BallDetector::detectBall(Mat color_image, Mat &original_image, int blue, int green, int red, BallDetectionArray &ball_detection_array)
{
  int label_area=400, circularity=70;
  param::get("ball_threshold/circularity", circularity);
  param::get("ball_threshold/label_area", label_area);
  short *label_matrix = new short[width_*height_];
  LabelingBS labeling;
  RegionInfoBS *region_info;

  labeling.Exec(color_image.data, label_matrix, width_, height_, true, label_area);

  int number_of_label = labeling.GetNumOfResultRegions();
  if (number_of_label > 100) number_of_label = 100;

  for(int i = 0; i < number_of_label; i++)
  {
    ColorBall color_ball(labeling.GetResultRegionInfo(i), blue, green, red);
    color_ball.calculateCircularity(label_matrix, width_, height_, i);
    if(color_ball.circularity()*100 >= circularity)
    {
      color_ball.addToTopic(ball_detection_array);
      color_ball.drawOnImage(original_image);
    }
  }
  delete label_matrix;
}

}
