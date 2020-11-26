#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "input_convertor");
  ros::NodeHandle nh_private("~");

  // Get params
  double frequency;
  nh_private.param("convertor_frequency", frequency, 20.0);

  // Main loop
  ros::Rate rate(frequency);
  ros::WallTime startTime = ros::WallTime::now();
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}