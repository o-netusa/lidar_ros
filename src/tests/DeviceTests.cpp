#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../lidar_ros_driver/LidarRosDriver.h"


ros::NodeHandle node;
TEST(DeviceTests, LidarRosDriver)
{    
    onet::lidar_ros::LidarRosDriver dvr(node);
    dvr.Start();
    EXPECT_TRUE(dvr.IsRunning());
    dvr.Stop();
    EXPECT_FALSE(dvr.IsRunning());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tests");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
