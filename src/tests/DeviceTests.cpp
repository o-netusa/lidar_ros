#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../lidar_ros_driver/LidarRosDriver.h"


TEST(LidarRosDriver, LidarRosDriver)
{    
    ros::NodeHandle node;
    onet::lidar_ros::LidarRosDriver dvr(node);
    dvr.Start();
    EXPECT_TRUE(dvr.IsRunning());
    dvr.Stop();
    EXPECT_FALSE(dvr.IsRunning());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tests");
    return RUN_ALL_TESTS();
}
