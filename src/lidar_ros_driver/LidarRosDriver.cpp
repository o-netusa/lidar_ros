/**************************************************************************
 * @file: RosServer.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include "LidarRosDriver.h"
#include <common/FileSystem.h>
#include <common/FileSystem.h>
#include <config/DeviceParamsConfig.h>
#include <DeviceManager.h>
#include <LidarDevice.h>
#include <PlaybackDevice.h>
#include <DolphinDevice.h>
#include <Types.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>


namespace onet { namespace lidar_ros {

static auto param_file = (fs::path(cppbase::filesystem::GetConfigDir()) /
                          onet::lidar::LIDAR_CHECK_FILE).string(); // default param file

class ViewerCallback  :public onet::lidar::DeviceCallback
{
public:
    void SetPublisher(ros::Publisher* cloud_pub)
    {
        m_cloud_pub=cloud_pub;
    }
    void HandlePointCloud(uint32_t frame_id, std::shared_ptr<onet::lidar::PointCloud> cloud,[[maybe_unused]]
     const std::string &file_name = {})
    {
        if (!cloud || m_cloud_pub==nullptr)
        {
            return;
        }
        sensor_msgs::PointCloud pointcloud;
        pointcloud.header.stamp=ros::Time::now();
        pointcloud.header.frame_id="sensor_frame";
        pointcloud.points.resize(cloud->size());
        pointcloud.channels.resize(2);
        pointcloud.channels[0].name="intensities";
        pointcloud.channels[0].values.resize(cloud->size());
        pointcloud.channels[1].name="rgb";
        pointcloud.channels[1].values.resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); i++)
        {
            const auto &pt = cloud->at(i);
            pointcloud.points[i].x = pt[0];
            pointcloud.points[i].y = pt[1];
            pointcloud.points[i].z = pt[2];
        }
        m_cloud_pub->publish(pointcloud);
    }
    void PlaybackDone() {}

    ros::Publisher* m_cloud_pub{nullptr};
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
    bool m_playback{false};
    std::thread m_set_thread;
    ros::NodeHandle m_node;      //节点
    ros::Publisher m_cloud_pub;  //点云发布者

    std::shared_ptr<ViewerCallback> m_viewcallback{nullptr};
    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};
    std::shared_ptr<onet::lidar::DlphDeviceParameter> m_dev_param;

    Impl(ros::NodeHandle node) : m_node(node)
    {
        m_cloud_pub=m_node.advertise<sensor_msgs::PointCloud>("cloud",100);
        //m_viewcallback=std::make_shared<ViewerCallback>();
        m_viewcallback->SetPublisher(&m_cloud_pub);
        onet::lidar::config::Deserialize(m_dev_param, param_file);
        // get parameters
        // e.g.
        // m_node.getParameter()
        // ...
    }
    void ConnectDevice()
    {
        try
        {
            XmlRpc::XmlRpcValue connect_xml;
            if(m_node.getParam("connect",connect_xml))
            {
                std::string ip=connect_xml["ip"];
                int port=connect_xml["port"];
                m_lidar_device=GetLidarDevice(ip,port);
            }
        }
        catch (ros::Exception &e)
        {
            ROS_ERROR("Error:%s",e.what());
            return;
        }
    }

    void DisconnectDevice()
    {
        if(!m_lidar_device) return;
        if(m_lidar_device->Stop())
        {
            m_lidar_device=nullptr;
        }
        else
        {
            ROS_INFO("Error:Failed to stop scanning on the LiDAR sensor.");
        }
    }

    void SetLaserParameter()
    {
        LaserParameter laserparam;
        try
        {
            XmlRpc::XmlRpcValue laserparam_xml;
            if(m_node.getParam("laser_parameter",laserparam_xml))
            {
                laserparam.level=static_cast<int>(laserparam_xml["level"]);
                laserparam.level=static_cast<int>(laserparam_xml["factor"]);
                laserparam.level=static_cast<int>(laserparam_xml["pulse_width"]);
            }
        }
        catch (ros::Exception &e)
        {
            ROS_ERROR("Error:%s",e.what());
            return;
        }

        try
        {
            m_lidar_device->SetLaser(laserparam);
        }
        catch (std::exception &e)
        {
            ROS_INFO("Error:%s",e.what());
        }
    }
    void SetEchoNumberParameter()
    {
        if(!m_lidar_device) return;
        int echo_number;
         if(m_node.getParam("echo_number",echo_number))
         {
             try
             {
                 m_lidar_device->SetEchoNumber(echo_number);
             }
             catch (std::exception &e)
             {
                 ROS_INFO("Error:%s",e.what());
             }
         }
    }

    void SetRawDataType()
    {
        try
        {
            int32_t type;
            if(m_node.getParam("raw_data_type",type))
            {
                m_lidar_device->SetRawDataType((RawDataType)type);
            }
        }
        catch (std::exception &e)
        {
            ROS_INFO("Error:%s",e.what());
        }
    }
    void SetScanMode()
    {
        try
        {
            int mode;
            if(m_node.getParam("scan_mode",mode))
            {
               m_lidar_device->SetScanMode((ScanMode)mode);
            }
        }
        catch (std::exception &e)
        {
           ROS_INFO("Error:%s",e.what());
        }
    }
    void SetViewParameter()
    {
        ViewParameter viewparam;
        try
        {
            XmlRpc::XmlRpcValue viewparam_xml;
            if(m_node.getParam("view_parameter",viewparam_xml) && viewparam_xml.getType()==XmlRpc::XmlRpcValue::TypeStruct)
            {
                viewparam.frame=static_cast<int>(viewparam_xml["frame"]);
                viewparam.steps[0]=static_cast<int>(viewparam_xml["steps"][0]);
                viewparam.steps[1]=static_cast<int>(viewparam_xml["steps"][1]);
                viewparam.steps[2]=static_cast<int>(viewparam_xml["steps"][2]);
                viewparam.steps[3]=static_cast<int>(viewparam_xml["steps"][3]);
                viewparam.perspectives[0]=static_cast<double>(viewparam_xml["perspectives"][0]);
                viewparam.perspectives[1]=static_cast<double>(viewparam_xml["perspectives"][1]);
                viewparam.perspectives[2]=static_cast<double>(viewparam_xml["perspectives"][2]);
                viewparam.perspectives[3]=static_cast<double>(viewparam_xml["perspectives"][3]);
                viewparam.perspectives[4]=static_cast<double>(viewparam_xml["perspectives"][4]);
            }
        }
        catch (ros::Exception &e)
        {
            ROS_INFO("Error:%s",e.what());
            return;
        }
        try
        {
            m_lidar_device->SetViewSpeed(viewparam);
        }
        catch (std::exception &e)
        {
            ROS_INFO("Error:%s",e.what());
        }
    }

    void SetPlayback()
    {
        if(m_playback_device && m_playback_device->IsStarted())
            return;
        std::vector<std::string> files;
        m_node.getParam("playback",files);
        if(files.size())
        {
            m_playback_device=GetPlaybackDevice(files);
            if(m_playback_device)
            {
                m_viewcallback->DisconnectAll();
                m_playback_device->SetParameter(m_dev_param);
                m_playback_device->Init();
                m_playback=true;
                m_viewcallback->per_frame_signal.connect([&](uint32_t,std::shared_ptr<onet::lidar::PointCloud>,const std::string &file_name){
                    //文件回放，文件列表滚动
                });
                m_viewcallback->play_done_signal.connect([&]() {
                    if(m_playback_device && !m_playback_device->IsStarted())
                    {
                        //文件回放，播放状态

                    }
                });
            }
        }
    }
    void Play()
    {
        int type;
        if(m_node.getParam("play",type))
        {
            switch (type)
            {
            case 1: //play
            {
                XmlRpc::XmlRpcValue option_xml;
                if(m_node.getParam("gather_parameter",option_xml))
                {
                    bool saveable=option_xml["savable"];
                    int rule=option_xml["FolderRule"];
                    std::string path=option_xml["path"];
                    lidar::WriteRawDataOption option(saveable,(lidar::WriteRawDataOption::FolderRule)rule,path);
                    if(m_playback)
                    {
                       if(m_playback_device)
                       {
                           if(m_playback_device->Start(m_viewcallback,option))
                           {
                               ROS_INFO("Error:Playback failed.");
                           }
                       }
                    }
                    else
                    {
                        if(m_lidar_device)
                        {
                            if(!m_lidar_device->Start(m_viewcallback,option))
                            {
                                ROS_INFO("Error:Failed to start scanning on the LiDAR sensor.");
                            }
                        }
                    }
                }
            }
                break;
            case 2: //pause
            {
                if(m_playback)
                {
                   if(m_playback_device && m_playback_device->IsStarted())
                   {
                       if(!m_playback_device->Stop())
                       {
                            ROS_INFO("Error:Playback stop failed");
                       }
                   }
                }
                else
                {
                    if(m_lidar_device && m_lidar_device->IsStarted())
                    {
                        if(!m_lidar_device->Stop())
                        {
                            ROS_INFO("Error:Lidar Device stop failed");
                        }
                    }
                }
            }
                break;
            case 0: //stop
            {
                if(m_playback)
                {
                   if(m_playback_device && m_playback_device->IsStarted())
                   {
                       if(m_playback_device->Stop())
                       {
                           m_playback_device=nullptr;
                       }
                   }
                }
                else
                {
                    if(m_lidar_device && m_lidar_device->IsStarted())
                    {
                        if(m_lidar_device->Stop())
                        {
                            m_lidar_device=nullptr;
                        }
                    }
                }
            }
                break;

            }
        }
    }
    void UpdateParameter()
    {
        std::string update_parameter;
        if(m_node.getParam("update_parameter",update_parameter))
        {
            if(update_parameter=="connect")
            {
                this->ConnectDevice();
            }
            else if(update_parameter=="laser_parameter")
            {
                this->SetLaserParameter();
            }
            else if(update_parameter=="echo_number")
            {
                this->SetEchoNumberParameter();
            }
            else if(update_parameter=="raw_data_type")
            {
                this->SetRawDataType();
            }
            else if(update_parameter=="scan_mode")
            {
                this->SetScanMode();
            }
            else if(update_parameter=="playback")
            {
                this->SetPlayback();
            }
            else if(update_parameter=="view_parameter")
            {
                this->SetViewParameter();
            }
            else if(update_parameter=="disconnect_device")
            {
                this->DisconnectDevice();
            }
            else if(update_parameter=="play")
            {
                this->Play();
            }
        }
    }

    void Start()
    {
        m_set_thread=std::thread([this]{
            this->m_running=true;
            ros::Rate loop_rate(20);
            while (this->m_running) {
                this->UpdateParameter();
                loop_rate.sleep();
            }
        });

    }
    void Stop()
    {
        if(m_set_thread.joinable())
        {
            m_running=false;
            m_set_thread.join();
        }
    }

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

void LidarRosDriver::Stop()
{
    m_impl->Stop();
}

}}  // namespace onet::lidar_ros
