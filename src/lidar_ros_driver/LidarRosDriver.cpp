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
#include <processing/PointCloudProcessing.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <thread>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <ctime>

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

    std::string m_param_dir{""};
    std::string m_point_cloud_topic_name{"lidar_point_cloud"};
    std::string m_frame_id{"lidar"};
    std::string m_device_ip{"192.168.1.2"};
    int m_port{2368};
    std::string m_playback_file_path;
    int m_playback_fps{10};

    std::function<void(uint32_t, onet::lidar::PointCloud<onet::lidar::PointXYZIT> &)> m_callback{
        nullptr};

    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};
    std::shared_ptr<onet::lidar::DlphDeviceParameter> m_dev_param;

    void InitLidar(ros::NodeHandle node)
    {
        std::string namespace_str = m_node.getNamespace();
        m_auto_start = m_node.param<bool>(namespace_str + "/onet_lidar_ros_driver/auto_start", m_auto_start);
        m_save_bag = m_node.param<bool>(namespace_str + "/onet_lidar_ros_driver/save_bag", m_save_bag);
        m_point_cloud_topic_name = m_node.param<std::string>(
            namespace_str + "/onet_lidar_ros_driver/point_cloud_topic_name", m_point_cloud_topic_name);
        m_device_ip = m_node.param<std::string>(namespace_str + "/onet_lidar_ros_driver/device_ip", m_device_ip);
        m_port = m_node.param<int>(namespace_str + "/onet_lidar_ros_driver/port", m_port);
        m_frame_id = m_node.param<std::string>(namespace_str + "/onet_lidar_ros_driver/frame_id", m_frame_id);
        m_playback_file_path = m_node.param<std::string>(
            namespace_str + "/onet_lidar_ros_driver/playback_file_path", m_playback_file_path);
        m_param_dir = m_node.param<std::string>(
            namespace_str + "/onet_lidar_ros_driver/param_path", m_param_dir);

        ROS_INFO("Config file path: %s", m_param_dir.c_str());
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

        m_callback = [this](uint32_t frame_id, lidar::PointCloud<lidar::PointXYZIT> &cloud) {
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

    void HandlePointCloud(uint32_t frame_id, lidar::PointCloud<onet::lidar::PointXYZIT> cloud)
    {
        if (cloud.empty())
        {
            ROS_ERROR("*******No points in cloud*********");
            return;
        }
        cppbase::Timer<cppbase::us> timer;

        // POINTXYZIRGBT
        sensor_msgs::PointCloud2 msg_pointcloud;

        int fields = 9;
        msg_pointcloud.fields.clear();
        msg_pointcloud.fields.reserve(fields);

        // send by row
        msg_pointcloud.width = cloud.size();
        msg_pointcloud.height = 1;

        int offset = 0;
        offset = addPointField(msg_pointcloud, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
        offset = addPointField(msg_pointcloud, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
        offset = addPointField(msg_pointcloud, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
        offset = addPointField(msg_pointcloud, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
        offset = addPointField(msg_pointcloud, "utc_time", 1, sensor_msgs::PointField::UINT32, offset);
        offset = addPointField(msg_pointcloud, "ms_time", 1, sensor_msgs::PointField::UINT32, offset);

        msg_pointcloud.point_step = offset;
        msg_pointcloud.row_step = msg_pointcloud.width * msg_pointcloud.point_step;
        msg_pointcloud.is_dense = true; // not containing NAN 
        msg_pointcloud.data.resize(msg_pointcloud.point_step * msg_pointcloud.width * msg_pointcloud.height);

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg_pointcloud, "intensity");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_utc_time(msg_pointcloud, "utc_time");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_ms_time(msg_pointcloud, "ms_time");

        
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            const auto &pt = cloud.at(i);
            *iter_x = pt[0];
            *iter_y = pt[1];
            *iter_z = pt[2];
            *iter_intensity = pt[3];
            *iter_utc_time = pt.utc;
            *iter_ms_time = pt.time_stamp;

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
            ++iter_utc_time;
            ++iter_ms_time;
        }

        /*FPGA返回的GPS时间不带有年月日，先使用boost库获取系统的年月日，
        **然后转换成struct tm，和GPS时间（时分秒毫秒微妙）进行拼接，
        **再转换成ros::Time
        **/
        boost::gregorian::date now_date = boost::gregorian::day_clock::universal_day();
        struct tm now_tm = boost::gregorian::to_tm(now_date);
        now_tm.tm_hour = ((cloud[0].utc & 0x1F000) >> 12) + 8;
        now_tm.tm_min = (cloud[0].utc & 0xFC0) >> 6;
        now_tm.tm_sec = cloud[0].utc & 0x3F;
        double now_sec = (mktime(&now_tm) * 1000000 + (cloud[0].time_stamp >> 10) * 1000 +
                        (cloud[0].time_stamp & 0x3FF)) /
                        1000000.0;
        msg_pointcloud.header.stamp = ros::Time(now_sec);
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
        ROS_INFO("Current directory: %s", fs::current_path().string().c_str());
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
                m_playback_device->SetConfigDir(m_param_dir);
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
                m_lidar_device->SetConfigDir(m_param_dir);
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
