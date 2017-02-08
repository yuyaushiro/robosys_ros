#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <robosys_ros/BallConfig.h>

void callbackBall(robosys_ros::BallConfig &config, uint32_t level)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ball_threshold");

  dynamic_reconfigure::Server<robosys_ros::BallConfig> ball_server;
  dynamic_reconfigure::Server<robosys_ros::BallConfig>::CallbackType r;

  r = boost::bind(&callbackBall, _1, _2);
  ball_server.setCallback(r);

  ros::spin();
  return 0;
}
