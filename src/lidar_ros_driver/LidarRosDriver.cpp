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
#include <boost/date_time/gregorian/gregorian.hpp>
#include <ctime>

#define USE_POINT_CLOUD_2_POINTXYZI 0

namespace onet { namespace lidar_ros {
	
struct Rgb
{
    Rgb(const float r, const float g, const float b)
    {
        m_r = static_cast<uint8_t>(r * 255);
        m_g = static_cast<uint8_t>(g * 255);
        m_b = static_cast<uint8_t>(b * 255);
        m_rgb = static_cast<uint32_t>((m_r << 16) + (m_g << 8) + m_b);
    }
    uint8_t m_r;
    uint8_t m_g;
    uint8_t m_b;
    uint32_t m_rgb;
};

Rgb color_table[256] = {
    Rgb(0, 0.501961, 1),  Rgb(0, 0.509725, 1),  Rgb(0, 0.51749, 1),    Rgb(0, 0.525255, 1),   Rgb(0, 0.53302, 1),
    Rgb(0, 0.540784, 1),  Rgb(0, 0.548549, 1),  Rgb(0, 0.556314, 1),   Rgb(0, 0.564078, 1),   Rgb(0, 0.571843, 1),
    Rgb(0, 0.579608, 1),  Rgb(0, 0.587373, 1),  Rgb(0, 0.595137, 1),   Rgb(0, 0.602902, 1),   Rgb(0, 0.610667, 1),
    Rgb(0, 0.618431, 1),  Rgb(0, 0.626196, 1),  Rgb(0, 0.633961, 1),   Rgb(0, 0.641725, 1),   Rgb(0, 0.64949, 1),
    Rgb(0, 0.657255, 1),  Rgb(0, 0.66502, 1),   Rgb(0, 0.672784, 1),   Rgb(0, 0.680549, 1),   Rgb(0, 0.688314, 1),
    Rgb(0, 0.696078, 1),  Rgb(0, 0.703843, 1),  Rgb(0, 0.711608, 1),   Rgb(0, 0.719373, 1),   Rgb(0, 0.727137, 1),
    Rgb(0, 0.734902, 1),  Rgb(0, 0.742667, 1),  Rgb(0, 0.750431, 1),   Rgb(0, 0.758196, 1),   Rgb(0, 0.765961, 1),
    Rgb(0, 0.773726, 1),  Rgb(0, 0.78149, 1),   Rgb(0, 0.789255, 1),   Rgb(0, 0.79702, 1),    Rgb(0, 0.804784, 1),
    Rgb(0, 0.812549, 1),  Rgb(0, 0.820314, 1),  Rgb(0, 0.828078, 1),   Rgb(0, 0.835843, 1),   Rgb(0, 0.843608, 1),
    Rgb(0, 0.851373, 1),  Rgb(0, 0.859137, 1),  Rgb(0, 0.866902, 1),   Rgb(0, 0.874667, 1),   Rgb(0, 0.882431, 1),
    Rgb(0, 0.890196, 1),  Rgb(0, 0.897961, 1),  Rgb(0, 0.905726, 1),   Rgb(0, 0.91349, 1),    Rgb(0, 0.921255, 1),
    Rgb(0, 0.92902, 1),   Rgb(0, 0.936784, 1),  Rgb(0, 0.944549, 1),   Rgb(0, 0.952314, 1),   Rgb(0, 0.960078, 1),
    Rgb(0, 0.967843, 1),  Rgb(0, 0.975608, 1),  Rgb(0, 0.983373, 1),   Rgb(0, 0.991137, 1),   Rgb(0, 1, 0.996078),
    Rgb(0, 1, 0.980392),  Rgb(0, 1, 0.964706),  Rgb(0, 1, 0.94902),    Rgb(0, 1, 0.933333),   Rgb(0, 1, 0.917647),
    Rgb(0, 1, 0.901961),  Rgb(0, 1, 0.886275),  Rgb(0, 1, 0.870588),   Rgb(0, 1, 0.854902),   Rgb(0, 1, 0.839216),
    Rgb(0, 1, 0.823529),  Rgb(0, 1, 0.807843),  Rgb(0, 1, 0.792157),   Rgb(0, 1, 0.776471),   Rgb(0, 1, 0.760784),
    Rgb(0, 1, 0.745098),  Rgb(0, 1, 0.729412),  Rgb(0, 1, 0.713726),   Rgb(0, 1, 0.698039),   Rgb(0, 1, 0.682353),
    Rgb(0, 1, 0.666667),  Rgb(0, 1, 0.65098),   Rgb(0, 1, 0.635294),   Rgb(0, 1, 0.619608),   Rgb(0, 1, 0.603922),
    Rgb(0, 1, 0.588235),  Rgb(0, 1, 0.572549),  Rgb(0, 1, 0.556863),   Rgb(0, 1, 0.541176),   Rgb(0, 1, 0.52549),
    Rgb(0, 1, 0.509804),  Rgb(0, 1, 0.494118),  Rgb(0, 1, 0.478431),   Rgb(0, 1, 0.462745),   Rgb(0, 1, 0.447059),
    Rgb(0, 1, 0.431373),  Rgb(0, 1, 0.415686),  Rgb(0, 1, 0.4),        Rgb(0, 1, 0.384314),   Rgb(0, 1, 0.368627),
    Rgb(0, 1, 0.352941),  Rgb(0, 1, 0.337255),  Rgb(0, 1, 0.321569),   Rgb(0, 1, 0.305882),   Rgb(0, 1, 0.290196),
    Rgb(0, 1, 0.27451),   Rgb(0, 1, 0.258824),  Rgb(0, 1, 0.243137),   Rgb(0, 1, 0.227451),   Rgb(0, 1, 0.211765),
    Rgb(0, 1, 0.196078),  Rgb(0, 1, 0.180392),  Rgb(0, 1, 0.164706),   Rgb(0, 1, 0.14902),    Rgb(0, 1, 0.133333),
    Rgb(0, 1, 0.117647),  Rgb(0, 1, 0.101961),  Rgb(0, 1, 0.0862745),  Rgb(0, 1, 0.0705882),  Rgb(0, 1, 0.054902),
    Rgb(0, 1, 0.0392157), Rgb(0, 1, 0.0235294), Rgb(0, 1, 0.00784314), Rgb(0.00784314, 1, 0), Rgb(0.0235294, 1, 0),
    Rgb(0.0392157, 1, 0), Rgb(0.054902, 1, 0),  Rgb(0.0705882, 1, 0),  Rgb(0.0862745, 1, 0),  Rgb(0.101961, 1, 0),
    Rgb(0.117647, 1, 0),  Rgb(0.133333, 1, 0),  Rgb(0.14902, 1, 0),    Rgb(0.164706, 1, 0),   Rgb(0.180392, 1, 0),
    Rgb(0.196078, 1, 0),  Rgb(0.211765, 1, 0),  Rgb(0.227451, 1, 0),   Rgb(0.243137, 1, 0),   Rgb(0.258824, 1, 0),
    Rgb(0.27451, 1, 0),   Rgb(0.290196, 1, 0),  Rgb(0.305882, 1, 0),   Rgb(0.321569, 1, 0),   Rgb(0.337255, 1, 0),
    Rgb(0.352941, 1, 0),  Rgb(0.368627, 1, 0),  Rgb(0.384314, 1, 0),   Rgb(0.4, 1, 0),        Rgb(0.415686, 1, 0),
    Rgb(0.431373, 1, 0),  Rgb(0.447059, 1, 0),  Rgb(0.462745, 1, 0),   Rgb(0.478431, 1, 0),   Rgb(0.494118, 1, 0),
    Rgb(0.509804, 1, 0),  Rgb(0.52549, 1, 0),   Rgb(0.541176, 1, 0),   Rgb(0.556863, 1, 0),   Rgb(0.572549, 1, 0),
    Rgb(0.588235, 1, 0),  Rgb(0.603922, 1, 0),  Rgb(0.619608, 1, 0),   Rgb(0.635294, 1, 0),   Rgb(0.65098, 1, 0),
    Rgb(0.666667, 1, 0),  Rgb(0.682353, 1, 0),  Rgb(0.698039, 1, 0),   Rgb(0.713726, 1, 0),   Rgb(0.729412, 1, 0),
    Rgb(0.745098, 1, 0),  Rgb(0.760784, 1, 0),  Rgb(0.776471, 1, 0),   Rgb(0.792157, 1, 0),   Rgb(0.807843, 1, 0),
    Rgb(0.823529, 1, 0),  Rgb(0.839216, 1, 0),  Rgb(0.854902, 1, 0),   Rgb(0.870588, 1, 0),   Rgb(0.886275, 1, 0),
    Rgb(0.901961, 1, 0),  Rgb(0.917647, 1, 0),  Rgb(0.933333, 1, 0),   Rgb(0.94902, 1, 0),    Rgb(0.964706, 1, 0),
    Rgb(0.980392, 1, 0),  Rgb(0.996078, 1, 0),  Rgb(1, 0.988235, 0),   Rgb(1, 0.972549, 0),   Rgb(1, 0.956863, 0),
    Rgb(1, 0.941176, 0),  Rgb(1, 0.92549, 0),   Rgb(1, 0.909804, 0),   Rgb(1, 0.894118, 0),   Rgb(1, 0.878431, 0),
    Rgb(1, 0.862745, 0),  Rgb(1, 0.847059, 0),  Rgb(1, 0.831373, 0),   Rgb(1, 0.815686, 0),   Rgb(1, 0.8, 0),
    Rgb(1, 0.784314, 0),  Rgb(1, 0.768627, 0),  Rgb(1, 0.752941, 0),   Rgb(1, 0.737255, 0),   Rgb(1, 0.721569, 0),
    Rgb(1, 0.705882, 0),  Rgb(1, 0.690196, 0),  Rgb(1, 0.67451, 0),    Rgb(1, 0.658824, 0),   Rgb(1, 0.643137, 0),
    Rgb(1, 0.627451, 0),  Rgb(1, 0.611765, 0),  Rgb(1, 0.596078, 0),   Rgb(1, 0.580392, 0),   Rgb(1, 0.564706, 0),
    Rgb(1, 0.54902, 0),   Rgb(1, 0.533333, 0),  Rgb(1, 0.517647, 0),   Rgb(1, 0.501961, 0),   Rgb(1, 0.486275, 0),
    Rgb(1, 0.470588, 0),  Rgb(1, 0.454902, 0),  Rgb(1, 0.439216, 0),   Rgb(1, 0.423529, 0),   Rgb(1, 0.407843, 0),
    Rgb(1, 0.392157, 0),  Rgb(1, 0.376471, 0),  Rgb(1, 0.360784, 0),   Rgb(1, 0.345098, 0),   Rgb(1, 0.329412, 0),
    Rgb(1, 0.313726, 0),  Rgb(1, 0.298039, 0),  Rgb(1, 0.282353, 0),   Rgb(1, 0.266667, 0),   Rgb(1, 0.25098, 0),
    Rgb(1, 0.235294, 0),  Rgb(1, 0.219608, 0),  Rgb(1, 0.203922, 0),   Rgb(1, 0.188235, 0),   Rgb(1, 0.172549, 0),
    Rgb(1, 0.156863, 0),  Rgb(1, 0.141176, 0),  Rgb(1, 0.12549, 0),    Rgb(1, 0.109804, 0),   Rgb(1, 0.0941176, 0),
    Rgb(1, 0, 0),         Rgb(1, 0, 0),         Rgb(1, 0, 0),          Rgb(1, 0, 0),          Rgb(1, 0, 0),
    Rgb(1, 0, 0)};

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

#if USE_POINT_CLOUD_2_POINTXYZI
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
#else
        sensor_msgs::PointCloud2 msg_pointcloud;
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
        pointcloud.points.resize(cloud.size());
		
		//make sure that pt[3] <= 1 and pt[3] >= 0 
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            pcl::PointXYZRGB p;
            const auto &pt = cloud.at(i);
            p.x = pt[0];
            p.y = pt[1];
            p.z = pt[2];
            int idx = static_cast<int>(pt[3] * 255.0f);
            p.r = color_table[idx].m_r;
            p.g = color_table[idx].m_g;
            p.b = color_table[idx].m_b;
            pointcloud.points.emplace_back(p);
        }

        pcl::toROSMsg(pointcloud, msg_pointcloud);

        /*FPGA返回的GPS时间不带有年月日，先使用boost库获取系统的年月日，
        **然后转换成struct tm，和GPS时间（时分秒毫秒微妙）进行拼接，
        **再转换成ros::Time
        **/
        boost::gregorian::date now_date = boost::gregorian::day_clock::universal_day();
        struct tm now_tm = boost::gregorian::to_tm(now_date);
        now_tm.tm_hour = (cloud[0].utc >> 12) + 8;
        now_tm.tm_min = (cloud[0].utc & 0xFC0) >> 6;
        now_tm.tm_sec = cloud[0].utc & 0x3F;
        double now_sec = (mktime(&now_tm) * 1000000 + (cloud[0].time_stamp >> 10) * 1000 +
                          (cloud[0].time_stamp & 0x3FF)) /
                         1000000.0;
        msg_pointcloud.header.stamp = ros::Time(now_sec);
        msg_pointcloud.header.frame_id = m_frame_id;

        m_cloud_pub.publish(msg_pointcloud);
#endif

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
