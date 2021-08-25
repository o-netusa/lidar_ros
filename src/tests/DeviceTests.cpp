/**************************************************************************
 * @file: DeviceTests.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include <ParameterFlag.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <common_msgs/ParameterMsg.h>
#include <lidar_ros_driver/LidarRosDriver.h>

#include <gtest/gtest.h>
#include <thread>

TEST(LidarRosDriver, LidarRosDriver)
{
    ros::NodeHandle node;
    onet::lidar_ros::LidarRosDriver dvr(node);
    ros::Rate loop_rate(100);
    ros::spinOnce();
    loop_rate.sleep();
    EXPECT_TRUE(dvr.IsRunning());
    EXPECT_FALSE(dvr.IsRunning());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tests");
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
