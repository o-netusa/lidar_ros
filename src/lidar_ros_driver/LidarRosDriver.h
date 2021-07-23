
/**************************************************************************
 * @file: RosServer.h
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#pragma once

#include <ros/ros.h>

#include <memory>

namespace onet { namespace lidar_ros {

class LidarRosDriver
{
public:
    LidarRosDriver(ros::NodeHandle node);
    ~LidarRosDriver() = default;

    void Start();
    bool IsRunning() const;

private:
    struct Impl;
    std::shared_ptr<Impl> m_impl;
};
}}  // namespace onet::lidar_ros
