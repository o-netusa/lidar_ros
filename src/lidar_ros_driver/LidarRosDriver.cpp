/**************************************************************************
 * @file: RosServer.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include "LidarRosDriver.h"

#include <DeviceManager.h>
#include <LidarDevice.h>
#include <PlaybackDevice.h>
#include <Types.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>

namespace onet { namespace lidar_ros {

class ViewerCallback  //:public onet::lidar::DeviceCallback
{
public:
    void SetVisualizer() {}
    // void HandlePointCloud(uint32_t frame_id, std::shared_ptr<PointCloud> cloud,[[maybe_unused]]
    // const std::string &file_name = {})
    //{
    //    if (!cloud)
    //    {
    //        return;
    //    }
    //}
    void PlaybackDone() {}
};

onet::lidar::PlaybackDevice *GetPlaybackDevice(const std::vector<std::string> &file_list)
{
    static uuids::uuid play_device_id = uuids::uuid();
    if (!play_device_id.is_nil())
    {
        onet::lidar::DeviceManager::GetInstance().RemoveDevice(play_device_id);
        play_device_id = uuids::uuid();
    }
    play_device_id = lidar::DeviceManager::GetInstance().CreateDevice(file_list);
    return dynamic_cast<lidar::PlaybackDevice *>(
        lidar::DeviceManager::GetInstance().GetDevice(play_device_id));
}

onet::lidar::LidarDevice *GetLidarDevice(const std::string &strIP, int port)
{
    static uuids::uuid lidar_device_id = uuids::uuid();
    if (lidar_device_id.is_nil())
    {
        onet::lidar::DeviceManager::GetInstance().RemoveDevice(lidar_device_id);
        lidar_device_id = uuids::uuid();
    }
    lidar_device_id = onet::lidar::DeviceManager::GetInstance().CreateDevice(strIP, port);
    return dynamic_cast<lidar::LidarDevice *>(
        lidar::DeviceManager::GetInstance().GetDevice(lidar_device_id));
}

struct LidarRosDriver::Impl
{
    bool m_running{false};
    ros::NodeHandle m_node;      //节点
    ros::Publisher m_cloud_pub;  //点云发布者

    ViewerCallback *m_viewcallback;
    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};

    Impl(ros::NodeHandle node) : m_node(node)
    {
        // get parameters
        // e.g.
        // m_node.getParameter()
        // ...
    }

    void Start() {}
};

LidarRosDriver::LidarRosDriver(ros::NodeHandle node)
    : m_impl(std::make_shared<LidarRosDriver::Impl>(node))
{}

void LidarRosDriver::Start()
{
    m_impl->Start();
}

bool LidarRosDriver::IsRunning() const
{
    return m_impl->m_running;
}

}}  // namespace onet::lidar_ros
