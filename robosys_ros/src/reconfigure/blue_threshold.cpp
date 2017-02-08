#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <robosys_ros/BlueConfig.h>

void callbackBlue(robosys_ros::BlueConfig &config, uint32_t level)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "blue_threshold");

  dynamic_reconfigure::Server<robosys_ros::BlueConfig> blue_server;
  dynamic_reconfigure::Server<robosys_ros::BlueConfig>::CallbackType r;

  r = boost::bind(&callbackBlue, _1, _2);
  blue_server.setCallback(r);

  ros::spin();
  return 0;
}
