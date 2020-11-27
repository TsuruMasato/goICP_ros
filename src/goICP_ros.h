#include "jly_goicp.h"

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/buffer_core.h>

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
  void convert_cloudmsg2_to_point3d(,);
  void set_object_cloud();
  void set_camera_cloud(sensor_msgs::PointCloud2 &input_msg);
  void callback_camera(const sensor_msgs::PointCloud2ConstPtr &input);
  void callback_initial_pose();

private:
  std::shared_ptr<ros::NodeHandle> nh_ptr_;
  std::mutex mutex_model_cloud_;
  std::mutex mutex_camera_cloud_;
  sensor_msgs::PointCloud2 object_cloud_;
  sensor_msgs::PointCloud2 input_camera_cloud_;
  POINT3D *pModel_, *pData_;
};
}