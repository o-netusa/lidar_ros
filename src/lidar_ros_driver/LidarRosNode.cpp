/**************************************************************************
 * @file: LidarRosNode.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include <ros/ros.h>

#include "LidarRosDriver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "onet_lidar_node");
    ros::NodeHandle node;
    onet::lidar_ros::LidarRosDriver dvr(node);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // Check running status
        bool ret = dvr.IsRunning();
        if (!ret)
            break;
        // dvr.UpdateParameter();
        // Run LidarDevice
        dvr.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
