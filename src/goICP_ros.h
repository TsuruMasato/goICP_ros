#include "jly_goicp.h"
#include <ros/ros.h>

namespace goICP_ros_namespace
{

class goICP_ros : public GoICP
{
public:
  goICP_ros(ros::NodeHandle &nh);
  ~goICP_ros(){};

  void setConfig();

  bool run(){};
  bool original_main(int argc, char **argv);

private:
  std::shared_ptr<ros::NodeHandle> nh_ptr_;
};
}