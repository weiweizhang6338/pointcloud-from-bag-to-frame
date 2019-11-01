/*
 * Copyright (C) 2019 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 */

#ifndef POINTCLOUD_FROM_BAG_TO_FRAME_POINTCLOUD_FROM_BAG_TO_FRAME_H_
#define POINTCLOUD_FROM_BAG_TO_FRAME_POINTCLOUD_FROM_BAG_TO_FRAME_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> /* pcl::PointCloud */
#include <pcl/point_types.h> /* pcl::PointXYZ and pcl::PointXYZI*/
#include <pcl_ros/point_cloud.h>

#include "type.h"

/**
 * @brief Extracts and saves the point cloud data from rosbag to bin or pcd
 * files.
 */
class PointcloudFromBagToFrameNode {
 public:
  PointcloudFromBagToFrameNode();

  void InCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& inMsg);

 private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_in_cloud_;

  /*------------- params settings --------------*/

  // input cloud topic name
  std::string in_cloud_topic_;

  // output file dir
  std::string out_file_dir_;

  // file interval saved
  int out_file_interval_;

  // file type
  int out_file_type_;

  // point type
  int out_point_type_;

  /*------------- variables --------------*/

  int num_file_received_;

  int num_file_saved_;

  bool is_saving_;

  /*------------- functions --------------*/

  void SavePointTypeCloud(const std::string& file_path, const int& file_type,
                          type::PointTypeCloudPtr cloud_ptr);

  void SavePointITypeCloud(const std::string& file_path, const int& file_type,
                           type::PointITypeCloudPtr cloud_ptr);
};

#endif  // POINTCLOUD_FROM_BAG_TO_FRAME_POINTCLOUD_FROM_BAG_TO_FRAME_H_