/**************************************************************************
 * @file: LidarNode.cpp
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
    dvr.Start();
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // Check running status
        bool ret = dvr.IsRunning();
        if (!ret)
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }
    //ROS_INFO("tuichu lidar_ros_driver");
    dvr.Stop();
    return 0;
}
