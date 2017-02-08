#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <robosys_ros/YellowConfig.h>

void callbackYellow(robosys_ros::YellowConfig &config, uint32_t level)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yellow_threshold");

  dynamic_reconfigure::Server<robosys_ros::YellowConfig> yellow_server;
  dynamic_reconfigure::Server<robosys_ros::YellowConfig>::CallbackType r;

  r = boost::bind(&callbackYellow, _1, _2);
  yellow_server.setCallback(r);

  ros::spin();
  return 0;
}
