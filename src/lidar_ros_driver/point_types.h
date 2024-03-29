/**************************************************************************
 * @file: point_types.h
 * @brief: Point Cloud Library point structures for onet lidar data
 *
 * Copyright (c) 2021 - present O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#pragma once

#include <pcl/point_types.h>

namespace onet { namespace lidar_ros {

struct PointXYZIT
{
    PCL_ADD_POINT4D;  // quad-word XYZ
    float intensity;  // point intensity reading
    uint32_t utc_time;                 // point time utc
    uint32_t ms_time;                 // point time ms
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;
}}

POINT_CLOUD_REGISTER_POINT_STRUCT(onet::lidar_ros::PointXYZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, utc_time, utc_time)
                                  (uint32_t, ms_time, ms_time))
