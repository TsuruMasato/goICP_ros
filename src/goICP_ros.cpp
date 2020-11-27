#include "goICP_ros.h"

/********************************************************************
Main Function for point cloud registration with Go-ICP Algorithm
Last modified: Feb 13, 2014

"Go-ICP: Solving 3D Registration Efficiently and Globally Optimally"
Jiaolong Yang, Hongdong Li, Yunde Jia
International Conference on Computer Vision (ICCV), 2013

Copyright (C) 2013 Jiaolong Yang (BIT and ANU)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#include <time.h>
#include <iostream>
#include <fstream>
using namespace std;
using namespace pcl;
using namespace goICP_ros_namespace;

#include "jly_goicp.h"
#include "ConfigMap.hpp"

#define DEFAULT_OUTPUT_FNAME "output.txt"
#define DEFAULT_CONFIG_FNAME "config.txt"
#define DEFAULT_MODEL_FNAME "model.txt"
#define DEFAULT_DATA_FNAME "data.txt"

goICP_ros::goICP_ros(ros::NodeHandle &nh)
             : nh_ptr_(std::make_shared<ros::NodeHandle>(nh))
{
  setConfig();
  ros::Subscriber sub_camera = nh_ptr_->subscribe<sensor_msgs::PointCloud2>("/online_object_detector/camera_cloud", 1, &goICP_ros::callback_camera, this);
  ros::Subscriber sub_object = nh_ptr_->subscribe<sensor_msgs::PointCloud2>("/online_object_detector/object_cloud", 1, &goICP_ros::callback_object, this);
}

void goICP_ros::callback_camera(const sensor_msgs::PointCloud2ConstPtr &input)
{
  std::lock_guard<std::mutex> lock(mutex_camera_cloud_);
  latest_camera_cloud_ = *input;
  is_camera_new_ = true;
}

void goICP_ros::callback_object(const sensor_msgs::PointCloud2ConstPtr &input)
{
  std::lock_guard<std::mutex> lock(mutex_object_cloud_);
  latest_object_cloud_ = *input;
  is_object_new_ = true;
}

void goICP_ros::setConfig()
{
  nh_ptr_->param("MSEThresh", MSEThresh, 0.001f);
  nh_ptr_->param("rotMinX", initNodeRot.a, -3.1416f);
  nh_ptr_->param("rotMinY", initNodeRot.b, -3.1416f);
  nh_ptr_->param("rotMinZ", initNodeRot.c, -3.1416f);
  nh_ptr_->param("rotWidth", initNodeRot.w, 6.2832f);
  nh_ptr_->param("transMinX", initNodeTrans.x, -0.5f);
  nh_ptr_->param("transMinY", initNodeTrans.y, -0.5f);
  nh_ptr_->param("transMinZ", initNodeTrans.z, -0.5f);
  nh_ptr_->param("transWidth", initNodeTrans.w, 1.0f);
  nh_ptr_->param("trimFraction", trimFraction, 0.0f);
  nh_ptr_->param("distTransSize", dt.SIZE, 300);
  nh_ptr_->param("distTransExpandFactor", dt.expandFactor, 2.0);
  // If < 0.1% trimming specified, do no trimming
  if (trimFraction < 0.001)
  {
    doTrim = false;
  }

  ROS_INFO("config:");
  ROS_INFO("MSEThresh : %f", MSEThresh);
  ROS_INFO("rotMinX : %f", initNodeRot.a);
  ROS_INFO("rotMinY : %f", initNodeRot.b);
  ROS_INFO("rotMinZ : %f", initNodeRot.c);
  ROS_INFO("rotWidth : %f", initNodeRot.w);
  ROS_INFO("transMinX : %f", initNodeTrans.x);
  ROS_INFO("transMinY : %f", initNodeTrans.y);
  ROS_INFO("transMinZ : %f", initNodeTrans.z);
  ROS_INFO("transWidth : %f", initNodeTrans.w);
  ROS_INFO("trimFraction : %f", trimFraction);
  ROS_INFO("distTransSize : %d", dt.SIZE);
  ROS_INFO("distTransExpandFactor : %f", dt.expandFactor);
  ROS_INFO("doTrim : %d", doTrim);
}

bool goICP_ros::original_main(int argc, char **argv)
{
  // check if it receives both data.
  if(!is_camera_new_ || !is_object_new_)
  {
    ROS_ERROR("is_camera_new_: %d, is_object_new_: %d", is_camera_new_, is_object_new_);
    return false;
  }
 
  // Set model and data point clouds
  set_camera_cloud(latest_camera_cloud_);  //update Nd in this func.
  set_object_cloud(latest_object_cloud_);  //update Nm in this func.

  // Build Distance Transform
  cout << "Building Distance Transform..." << flush;
  clockBegin_ = clock();
  BuildDT();
  clockEnd_ = clock();
  ROS_INFO("BuildDT took %f s (CPU)", (double)(clockEnd_ - clockBegin_) / CLOCKS_PER_SEC);

  // Run GO-ICP
  //if (NdDownsampled > 0)
  //{
  //  Nd = NdDownsampled; // Only use first NdDownsampled data points (assumes data points are randomly ordered)
  //}

  clockBegin_ = clock();
  Register();
  clockEnd_ = clock();
  double time = (double)(clockEnd_ - clockBegin_) / CLOCKS_PER_SEC;
  cout << "Optimal Rotation Matrix:" << endl;
  cout << optR << endl;
  cout << "Optimal Translation Vector:" << endl;
  cout << optT << endl;
  ROS_INFO("Finished in %f", time);

  ofstream ofile;
  ofile.open(outputFname_.c_str(), ofstream::out);
  ofile << time << endl;
  ofile << optR << endl;
  ofile << optT << endl;
  ofile.close();

  delete (pModel);  //TODO: reset the cloud data in update function.
  delete (pData);
  return true;
}

void goICP_ros::set_camera_cloud(sensor_msgs::PointCloud2 &input_msg)
{
  if(!is_camera_new_)
  {
    return;
  }

  pcl::PointCloud<PointXYZRGB>::Ptr input_pc(new pcl::PointCloud<PointXYZRGB>);
  
  {
    std::lock_guard<std::mutex> lock(mutex_camera_cloud_);
    pcl::fromROSMsg(input_msg, *input_pc);
  }
  
  Nd = input_pc->size();

  if (Nd <= 0)
  {
    return;
  }
  //delete(pData);
  pData = (POINT3D *)malloc(sizeof(POINT3D) * Nd);
  for (size_t i = 0; i < Nd; i++)
  {
    pData[i].x = input_pc->at(i).x;
    pData[i].y = input_pc->at(i).y;
    pData[i].z = input_pc->at(i).z;
  }
  is_camera_new_ = true;
}

void goICP_ros::set_object_cloud(sensor_msgs::PointCloud2 &input_msg)
{
  if (!is_object_new_)
  {
    return;
  }

  pcl::PointCloud<PointXYZRGB>::Ptr input_pc(new pcl::PointCloud<PointXYZRGB>);

  {
    std::lock_guard<std::mutex> lock(mutex_object_cloud_);
    pcl::fromROSMsg(input_msg, *input_pc);
  }

  Nm = input_pc->size();

  if (Nm <= 0)
  {
    return;
  }
  //delete(pModel);
  pModel = (POINT3D *)malloc(sizeof(POINT3D) * Nm);
  for (size_t i = 0; i < Nm; i++)
  {
    pModel[i].x = input_pc->at(i).x;
    pModel[i].y = input_pc->at(i).y;
    pModel[i].z = input_pc->at(i).z;
  }
  is_object_new_ = true;
}