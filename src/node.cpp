#include <ros/ros.h>
#include "goICP_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goICP_ros");
  ros::NodeHandle nh_private("~");

  // Get params
  double frequency;
  nh_private.param("frequency_max", frequency, 2.0);

  goICP_ros_namespace::goICP_ros goICP_ros_(nh_private);
  //goICP_ros_.set_object_cloud();  //TODO: update Nm referencing to the input.

  // Main loop
  ros::Rate rate(frequency);
  ros::WallTime startTime = ros::WallTime::now();
  while (ros::ok())
  {
    ros::spinOnce();
    goICP_ros_.run();
    rate.sleep();
  }
  return 0;
}