#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <robosys_ros/BallDetection.h>
#include <robosys_ros/BallDetectionArray.h>
#include "robosys_ros/Labeling.h"


namespace robosys_ros{

class ColorBall
{
 public:
  ColorBall(RegionInfoBS *region_info, int blue, int green, int red);
  ~ColorBall();
  void calculateCircularity(short *label_matrix, int width, int height, int i);
  double circularity();
  void addToTopic(BallDetectionArray &ball_detection_array);
  void drawOnImage(cv::Mat &original_image);
 private:
  void setRegionInformation();

 private:
  int blue_, green_, red_;
  double top_x_, top_y_;
  double bottom_x_, bottom_y_;
  double left_x_, left_y_;
  double right_x_, right_y_;
  double center_x_, center_y_;
  double area_, length_;
  double circularity_;
  RegionInfoBS* region_info_;
};

}
