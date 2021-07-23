/**************************************************************************
 * @file:  PointCloudIO.h
 * @brief:
 *
 * Copyright (c) 2020-present O-Net Communications (ShenZhen) Limited.
 * All rights reserved.
 *
 *************************************************************************/

#pragma once

#include <common/Global.h>

#include <memory>
#include <string>
#include <vector>

#include "DeviceParams.h"

namespace open3d { namespace geometry {
class PointCloud;
}}  // namespace open3d::geometry

namespace onet { namespace lidar {

/**
 * @brief Each point cloud data corresponds to a frame.
 *   An exception about IOIssue may be thrown.
 */
void DLLEXPORT ReadPointCloudsFromSource(const std::string &filename,
                                         DlphDeviceParameter &dev_param, DlphFileHeader &dev_info,
                                         std::vector<std::shared_ptr<PointCloud>> &pointclouds);

// bin file
/**
 * @brief Read point cloud data from .bin
 */
void DLLEXPORT ReadPointCloudFromBin(const std::string &filename, PointCloud &pointcloud);
/**
 * @brief Write point cloud data to .bin
 *   Make sure the filename is valid and the path is existing
 */
void DLLEXPORT WritePointCloudToBin(const std::string &filename, const PointCloud &pointcloud);

}}  // namespace onet::lidar
