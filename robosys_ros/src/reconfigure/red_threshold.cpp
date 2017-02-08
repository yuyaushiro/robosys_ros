#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <robosys_ros/RedConfig.h>

void callbackRed(robosys_ros::RedConfig &config, uint32_t level)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "red_threshold");

  dynamic_reconfigure::Server<robosys_ros::RedConfig> red_server;
  dynamic_reconfigure::Server<robosys_ros::RedConfig>::CallbackType r;

  r = boost::bind(&callbackRed, _1, _2);
  red_server.setCallback(r);

  ros::spin();
  return 0;
}
