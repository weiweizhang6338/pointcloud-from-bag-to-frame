/*
 * Copyright (C) 2019 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 */

#include "pointcloud_from_bag_to_frame_node.h"

PointcloudFromBagToFrameNode::PointcloudFromBagToFrameNode() : nh_("~") {
  nh_.param<std::string>("in_cloud_topic", in_cloud_topic_, "in_cloud");
  nh_.param<std::string>("out_file_dir", out_file_dir_, "~/point_cloud/");
  nh_.param<int>("out_file_interval", out_file_interval_, 1);
  nh_.param<int>("out_file_type", out_file_type_, 0);
  nh_.param<int>("out_point_type", out_point_type_, 0);

  sub_in_cloud_ = nh_.subscribe(
      in_cloud_topic_, 2, &PointcloudFromBagToFrameNode::InCloudCallback, this);

  num_file_received_ = 0;
  num_file_saved_ = 0;
  is_saving_ = false;
}

void PointcloudFromBagToFrameNode::InCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& inMsg) {
  ROS_INFO(
      "[PointcloudFromBagToFrameNode::InCloudCallback]: num file received: %d",
      ++num_file_received_);

  if (num_file_received_ % out_file_interval_ == 0) {
    if (is_saving_) {
      ROS_WARN(
          "[PointcloudFromBagToFrameNode::InCloudCallback]: point cloud data "
          "conflict, please slow down the rate of playing the rosbag.");
      return;
    } else {
      is_saving_ = true;
    }
    char ss[10];
    sprintf(ss, "%06d", num_file_saved_);
    std::string file_path;
    if (out_file_type_ == type::PointcloudFileType::BIN) {
      file_path = out_file_dir_ + std::string(ss) + std::string(".bin");
    } else if (out_file_type_ == type::PointcloudFileType::PCD) {
      file_path = out_file_dir_ + std::string(ss) + std::string(".pcd");
    } else {
      ROS_WARN(
          "[PointcloudFromBagToFrameNode::InCloudCallback]: output point cloud "
          "file type does not support.");
      is_saving_ = false;
      return;
    }
    type::PointTypeCloudPtr point_type_cloud_ptr(new type::PointTypeCloud);
    type::PointITypeCloudPtr pointi_type_cloud_ptr(new type::PointITypeCloud);
    if (out_point_type_ == type::PointcloudPointType::XYZ) {
      pcl::fromROSMsg(*inMsg, *point_type_cloud_ptr);
      SavePointTypeCloud(file_path, out_file_type_, point_type_cloud_ptr);
    } else if (out_point_type_ == type::PointcloudPointType::XYZI) {
      pcl::fromROSMsg(*inMsg, *pointi_type_cloud_ptr);
      SavePointITypeCloud(file_path, out_file_type_, pointi_type_cloud_ptr);
    } else {
      ROS_WARN(
          "[PointcloudFromBagToFrameNode::InCloudCallback]: output point cloud "
          "point type does not support.");
      is_saving_ = false;
      return;
    }
    num_file_saved_++;
    is_saving_ = false;
  }
}

void PointcloudFromBagToFrameNode::SavePointTypeCloud(
    const std::string& file_path, const int& file_type,
    type::PointTypeCloudPtr cloud_ptr) {
  if (file_type == type::PointcloudFileType::BIN) {
    std::fstream output(file_path.c_str(), std::ios::out | std::ios::binary);
    if (!output.good()) {
      ROS_INFO(
          "[PointcloudFromBagToFrameNode::SavePointcloud]: Can not open file: "
          "%s.",
          file_path.c_str());
      return;
    }
    output.seekg(0, std::ios::beg);
    for (type::PointType point : cloud_ptr->points) {
      output.write((char*)&point.x, sizeof(float));
      output.write((char*)&point.y, sizeof(float));
      output.write((char*)&point.z, sizeof(float));
    }
    output.close();
  } else if (file_type == type::PointcloudFileType::PCD) {
    pcl::io::savePCDFileASCII(file_path, *cloud_ptr);
  } else {
    ROS_INFO(
        "[PointcloudFromBagToFrameNode::SavePointCloud]: output point cloud "
        "file type does not support.");
  }
}

void PointcloudFromBagToFrameNode::SavePointITypeCloud(
    const std::string& file_path, const int& file_type,
    type::PointITypeCloudPtr cloud_ptr) {
  if (file_type == type::PointcloudFileType::BIN) {
    std::fstream output(file_path.c_str(), std::ios::out | std::ios::binary);
    if (!output.good()) {
      ROS_INFO(
          "[PointcloudFromBagToFrameNode::SavePointcloud]: Can not open file: "
          "%s.",
          file_path.c_str());
      return;
    }
    output.seekg(0, std::ios::beg);
    for (type::PointIType point : cloud_ptr->points) {
      output.write((char*)&point.x, sizeof(float));
      output.write((char*)&point.y, sizeof(float));
      output.write((char*)&point.z, sizeof(float));
      output.write((char*)&point.intensity, sizeof(float));
    }
    output.close();
  } else if (file_type == type::PointcloudFileType::PCD) {
    pcl::io::savePCDFileASCII(file_path, *cloud_ptr);
  } else {
    ROS_INFO(
        "[PointcloudFromBagToFrameNode::SavePointCloud]: output point cloud "
        "file type does not support.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_from_bag_to_frame_node");
  PointcloudFromBagToFrameNode pointcloud_from_bag_to_frame_node;
  ros::spin();

  return 0;
}
