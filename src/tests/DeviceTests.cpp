/**************************************************************************
 * @file: DeviceTests.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include <gtest/gtest.h>
#include <lidar_ros_driver/LidarRosDriver.h>
#include <ros/ros.h>

#include <thread>

TEST(LidarRosDriver, LidarRosDriver)
{
    ros::NodeHandle node;
    onet::lidar_ros::LidarRosDriver dvr(node);
    ros::Rate loop_rate(100);
    ros::spinOnce();
    loop_rate.sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tests");
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
