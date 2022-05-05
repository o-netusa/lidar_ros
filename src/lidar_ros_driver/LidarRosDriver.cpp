/**************************************************************************
 * @file: LidarRosDriver.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include "LidarRosDriver.h"

#include <DeviceManager.h>
#include <DolphinDevice.h>
#include <LidarDevice.h>
#include <PlaybackDevice.h>
#include <Types.h>
#include <XmlRpcValue.h>
#include <common/FileSystem.h>
#include <common/Timer.h>
#include <config/DeviceParamsConfig.h>
#include <pcl_conversions/pcl_conversions.h>
#include <processing/PointCloudProcessing.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>

#include <thread>

namespace onet { namespace lidar_ros {

static onet::lidar::PlaybackDevice *GetPlaybackDevice(const std::vector<std::string> &file_list)
{
    static uuids::uuid play_device_id{};
    if (!play_device_id.is_nil())
    {
        onet::lidar::DeviceManager::GetInstance().RemoveDevice(play_device_id);
    }
    play_device_id = lidar::DeviceManager::GetInstance().CreateDevice(file_list);
    return dynamic_cast<lidar::PlaybackDevice *>(
        lidar::DeviceManager::GetInstance().GetDevice(play_device_id));
}

static onet::lidar::LidarDevice *GetLidarDevice(const std::string &strIP, int port)
{
    static uuids::uuid lidar_device_id{};
    if (!lidar_device_id.is_nil())
    {
        onet::lidar::DeviceManager::GetInstance().RemoveDevice(lidar_device_id);
    }
    lidar_device_id = onet::lidar::DeviceManager::GetInstance().CreateDevice(strIP, port);
    return dynamic_cast<lidar::LidarDevice *>(
        lidar::DeviceManager::GetInstance().GetDevice(lidar_device_id));
}

struct LidarRosDriver::Impl
{
    bool m_auto_start{true};
    bool m_save_bag{false};
    std::string m_update_parameter;
    rosbag::Bag m_bag;
    ros::NodeHandle m_node;        //节点
    ros::Publisher m_cloud_pub;    //点云发布者
    ros::Publisher m_param_pub;    //参数设置状态发布者

    std::string m_point_cloud_topic_name{"lidar_point_cloud"};
    std::string m_frame_id{"lidar"};
    std::string m_device_ip{"192.168.1.2"};
    int m_port{2368};
    std::string m_playback_file_path;
    int m_playback_fps{10};

    std::function<void(uint32_t, onet::lidar::PointCloud<onet::lidar::PointXYZI> &)> m_callback{
        nullptr};

    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};
    std::shared_ptr<onet::lidar::DlphDeviceParameter> m_dev_param;

    void InitLidar(ros::NodeHandle node)
    {
        m_auto_start = m_node.param<bool>("/onet_lidar_ros_driver/auto_start", m_auto_start);
        m_save_bag = m_node.param<bool>("/onet_lidar_ros_driver/save_bag", m_save_bag);
        m_point_cloud_topic_name = m_node.param<std::string>(
            "/onet_lidar_ros_driver/point_cloud_topic_name", m_point_cloud_topic_name);
        m_device_ip = m_node.param<std::string>("/onet_lidar_ros_driver/device_ip", m_device_ip);
        m_port = m_node.param<int>("/onet_lidar_ros_driver/port", m_port);
        m_frame_id = m_node.param<std::string>("/onet_lidar_ros_driver/frame_id", m_frame_id);
        m_playback_file_path = m_node.param<std::string>(
            "/onet_lidar_ros_driver/playback_file_path", m_playback_file_path);

    }

    Impl(ros::NodeHandle node) : m_node(node)
    {
        try
        {
            // fetch parameters
            InitLidar(m_node);

        } catch (const std::exception &e)
        {
            ROS_ERROR("Error fetching parameters: %s", e.what());
        }
        m_cloud_pub = m_node.advertise<sensor_msgs::PointCloud2>(m_point_cloud_topic_name, 100);

        m_callback = [this](uint32_t frame_id, lidar::PointCloud<lidar::PointXYZI> &cloud) {
            HandlePointCloud(frame_id, cloud);
        };
        if (m_save_bag)
        {
            m_bag.open("test.bag", rosbag::bagmode::Write);
        }
        if (m_auto_start)
        {
            Run();
        }
    }

    ~Impl()
    {
        if (m_lidar_device)
        {
            ROS_INFO("Stop lidar device");
            m_lidar_device->Stop();
        }
        if (m_playback_device)
        {
            ROS_INFO("Stop playback device");
            m_playback_device->Stop();
        }
        if (m_save_bag)
        {
            m_bag.close();
        }
    }

    void HandlePointCloud(uint32_t frame_id, lidar::PointCloud<onet::lidar::PointXYZI> cloud)
    {
        if (cloud.empty())
        {
            return;
        }
        cppbase::Timer<cppbase::us> timer;
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        pointcloud.points.resize(cloud.size());
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            const auto &pt = cloud.at(i);
            pointcloud.points[i].x = pt[0];
            pointcloud.points[i].y = pt[1];
            pointcloud.points[i].z = pt[2];
            pointcloud.points[i].intensity = pt[3];
        }
        sensor_msgs::PointCloud2 msg_pointcloud;
        pcl::toROSMsg(pointcloud, msg_pointcloud);
        msg_pointcloud.header.stamp = ros::Time::now();
        msg_pointcloud.header.frame_id = m_frame_id;

        m_cloud_pub.publish(msg_pointcloud);
        ROS_INFO("end time:%d us", static_cast<int>(timer.Elapsed()));
        timer.Stop();
       
        if (m_save_bag)
        {
            m_bag.write(m_point_cloud_topic_name, ros::Time::now(), msg_pointcloud);
        }
    }

    /**
     * @brief Run the device directly
     */
    void Run()
    {
        ROS_INFO("Current directory: %s",fs::current_path().string().c_str());
        ROS_INFO("Playback file path: %s", m_playback_file_path.c_str());
        // Use PlaybackDevice if playback_file_path is not empty
        if (!m_playback_file_path.empty())
        {
            std::vector<std::string> file_list{m_playback_file_path};  // will improve later
            m_playback_device = GetPlaybackDevice(file_list);
            if (!m_playback_device)
            {
                ROS_ERROR("Error: failed to create playback device!");
                return;
            }
            try
            {
                m_playback_device->Init();
                m_playback_device->SetFPS(10);
                m_playback_device->RegisterPointCloudCallback(m_callback);
                if (!m_playback_device->Start())
                {
                    ROS_ERROR("Error: failed to start playback device!");
                    return;
                }
            } catch (std::exception &e)
            {
                ROS_ERROR("Error:%s", e.what());
            }
        } else
        {
            m_lidar_device = GetLidarDevice(m_device_ip, m_port);
            if (!m_lidar_device)
            {
                ROS_ERROR("Error: failed to connect to LiDAR device!");
                return;
            }
            try
            {
                m_lidar_device->Init();

                m_lidar_device->RegisterPointCloudCallback(m_callback);
                if (!m_lidar_device->Start())
                {
                    ROS_ERROR("Error: failed to start LiDAR device!");
                    return;
                }
            } catch (std::exception &e)
            {
                ROS_ERROR("Error:%s", e.what());
            }
        }
    }
};

LidarRosDriver::LidarRosDriver(ros::NodeHandle node)
    : m_impl(std::make_shared<LidarRosDriver::Impl>(node))
{}

}}  // namespace onet::lidar_ros
