#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include "robosys_ros/color_ball.h"
#include "robosys_ros/Labeling.h"

using namespace std;
using namespace cv;


namespace robosys_ros{


ColorBall::ColorBall(RegionInfoBS *region_info, int blue, int green, int red)
  :region_info_(region_info)
  ,blue_(blue)
  ,green_(green)
  ,red_(red)
  ,top_x_(0),top_y_(0)
  ,bottom_x_(0),bottom_y_(0)
  ,left_x_(0),left_y_(0)
  ,right_x_(0),right_y_(0)
  ,center_x_(0),center_y_(0)
  ,area_(0),length_(1)
  ,circularity_(0)
{
  ColorBall::setRegionInformation();
}

ColorBall::~ColorBall()
{
}

void ColorBall::calculateCircularity(short *label_matrix, int width, int height, int i)
{
  Mat label_extraction_matrix = Mat::zeros(Size(width, height), CV_8UC1);
  for(int y = top_y_-2; y <= bottom_y_+2; y++)
  {
    for(int x = left_x_-2; x <= right_x_+2; x++)
    {
      int a = width*y + x;
      if(label_matrix[a] == i+1)
      {
        label_extraction_matrix.data[label_extraction_matrix.step*y + x] = 255;
      }
    }
  }
  vector<vector<Point> > contours;
  findContours(label_extraction_matrix, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  area_ = contourArea(contours.at(0));
  length_ = arcLength(contours[0], true);
  circularity_ = (4 * M_PI * area_) / (length_ * length_);

  contours.clear();
}

double ColorBall::circularity()
{
  return circularity_;
}

void ColorBall::addToTopic(BallDetectionArray &ball_detection_array)
{
  BallDetection ball_detection;
  ball_detection.x = center_x_;
  ball_detection.y = center_y_;
  if(red_ == 255) ball_detection.color = 1;
  if(blue_ == 255) ball_detection.color = 2;
  if(green_ == 255) ball_detection.color = 3;

  ball_detection_array.detections.push_back(ball_detection);
}

void ColorBall::drawOnImage(Mat &original_image)
{
  rectangle(original_image,
      Point((int)left_x_, (int)top_y_),
      Point((int)right_x_, (int)bottom_y_),
      Scalar(blue_, green_, red_), 2, 4);
  rectangle(original_image,
      Point((int)center_x_-1, (int)center_y_-1),
      Point((int)center_x_+1, (int)center_y_+1),
      Scalar(0, 0, 0), 2, 4);

  stringstream circularity; circularity << "C: " << setprecision(3)
      << circularity_;
  putText(original_image, circularity.str(), Point(left_x_-5, top_y_-5),
      FONT_HERSHEY_SIMPLEX, 0.8, Scalar(blue_, green_, red_), 1, CV_AA);
}

void ColorBall::setRegionInformation()
{
  int left_x, top_y, right_x, bottom_y;
  float center_x, center_y;
  region_info_->GetMin(left_x, top_y);
  region_info_->GetMax(right_x, bottom_y);
  region_info_->GetCenter(center_x, center_y);
  left_x_ = left_x;
  top_y_ = top_y;
  right_x_ = right_x;
  bottom_y_ = bottom_y;
  center_x_ = center_x;
  center_y_ = center_y;
}

}
