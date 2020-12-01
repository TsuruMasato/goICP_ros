#include "jly_goicp.h"

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/buffer_core.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace goICP_ros_namespace
{

class goICP_ros : public GoICP
{
public:
  goICP_ros(ros::NodeHandle &nh);
  ~goICP_ros(){};

  void setConfig();

  bool run();
  void set_camera_cloud(sensor_msgs::PointCloud2 &input_msg);
  void set_object_cloud(sensor_msgs::PointCloud2 &input_msg);
  void callback_camera(const sensor_msgs::PointCloud2ConstPtr &input);
  void callback_object(const sensor_msgs::PointCloud2ConstPtr &input);
  //void callback_initial_pose();

private:
  bool is_camera_new_ = false;
  bool is_object_new_ = false;
  clock_t clockBegin_, clockEnd_;
  string outputFname_ = string("output_tsuru.txt");

  std::shared_ptr<ros::NodeHandle> nh_ptr_;
  std::mutex mutex_object_cloud_;
  std::mutex mutex_camera_cloud_;
  sensor_msgs::PointCloud2 latest_object_cloud_;
  sensor_msgs::PointCloud2 latest_camera_cloud_;
  ros::Subscriber sub_camera_;
  ros::Subscriber sub_object_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  ros::Publisher pub_pointcloud_;

  geometry_msgs::TransformStamped convert_result_into_TFmsg(Matrix input_R, Matrix input_T);
  Eigen::Quaternionf convert_MatrixR_into_Quat(Matrix input_R);
  POINT3D* get_pDataTempICP();
  void convert_POINT3D_to_PointCloudmsg(POINT3D *input, pcl::PointCloud<pcl::PointXYZRGB> &output);
};
}