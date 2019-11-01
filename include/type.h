/*
 * Copyright (C) 2019 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 */

#ifndef POINTCLOUD_FROM_BAG_TO_FRAME_TYPE_H_
#define POINTCLOUD_FROM_BAG_TO_FRAME_TYPE_H_

#include <pcl/point_cloud.h> /* pcl::PointCloud */
#include <pcl/point_types.h> /* pcl::PointXYZ and pcl::PointXYZI*/

namespace type {

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointTypeCloud;
typedef PointTypeCloud::Ptr PointTypeCloudPtr;
typedef pcl::PointXYZI PointIType;
typedef pcl::PointCloud<PointIType> PointITypeCloud;
typedef PointITypeCloud::Ptr PointITypeCloudPtr;

/**
 * @brief The file type of point cloud data.
 */
enum PointcloudFileType {
  BIN = 0,
  PCD,
};

/**
 * @brief The point type of point cloud data.
 */
enum PointcloudPointType {
  XYZ = 0,
  XYZI,
};

}  // namespace type

#endif  // POINTCLOUD_FROM_BAG_TO_FRAME_TYPE_H_