/**************************************************************************
 * @file: RosServer.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include "LidarRosDriver.h"
#include "ParameterFlag.h"
#include <common/FileSystem.h>
#include <common/Timer.h>
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
#include <thread>

#include <common_msgs/ParameterMsg.h>

namespace onet { namespace lidar_ros {

static auto param_file = (fs::path(cppbase::filesystem::GetConfigDir()) /
                          onet::lidar::LIDAR_CHECK_FILE).string(); // default param file

class ViewerCallback  :public onet::lidar::DeviceCallback
{
public:
    void SetPublisher(ros::Publisher *cloud_pub) { m_cloud_pub = cloud_pub; }
    void HandlePointCloud(uint32_t frame_id, std::shared_ptr<onet::lidar::PointCloud> cloud,
                          [[maybe_unused]] const std::string &file_name = {})
    {
        if (!cloud || m_cloud_pub == nullptr)
        {
            return;
        }
        cppbase::Timer<cppbase::us> timer;
        sensor_msgs::PointCloud pointcloud;
        pointcloud.header.stamp = ros::Time::now();
        pointcloud.header.frame_id = "sensor_frame";
        pointcloud.points.resize(cloud->size());
        pointcloud.channels.resize(2);
        pointcloud.channels[0].name = "intensities";
        pointcloud.channels[0].values.resize(cloud->size());
        pointcloud.channels[1].name = "rgb";
        pointcloud.channels[1].values.resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); i++)
        {
            const auto &pt = cloud->at(i);
            pointcloud.points[i].x = pt[0];
            pointcloud.points[i].y = pt[1];
            pointcloud.points[i].z = pt[2];
        }
        ROS_INFO("end time:%d us", static_cast<int>(timer.Elapsed()));
        timer.Stop();
        m_cloud_pub->publish(pointcloud);
    }
    void PlaybackDone() {}

    ros::Publisher* m_cloud_pub{nullptr};
};

onet::lidar::PlaybackDevice *GetPlaybackDevice(const std::vector<std::string> &file_list)
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

onet::lidar::LidarDevice *GetLidarDevice(const std::string &strIP, int port)
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
    bool m_running{false};
    bool m_playback{false};
    std::thread m_set_thread;
    ros::NodeHandle m_node;      //节点
    ros::Publisher m_cloud_pub;  //点云发布者
    ros::Publisher m_param_pub;  //参数设置状态发布者
    std::shared_ptr<ViewerCallback> m_viewcallback{nullptr};
    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};
    std::shared_ptr<onet::lidar::DlphDeviceParameter> m_dev_param;
    Impl(ros::NodeHandle node) : m_node(node)
    {
        m_cloud_pub=m_node.advertise<sensor_msgs::PointCloud>(pointcloud_msgs,100);
        m_param_pub=m_node.advertise<common_msgs::ParameterMsg>(param_msgs,100);
        m_viewcallback=std::make_shared<ViewerCallback>();
        m_viewcallback->SetPublisher(&m_cloud_pub);
        onet::lidar::config::Deserialize(m_dev_param, param_file);
        ClearParameter();
    }
    void SendParameterState(std::string parameter_flag,bool state,std::string error_info)
    {
        common_msgs::ParameterMsg msgs;
        msgs.parameter_flag=parameter_flag;
        msgs.state=state;
        msgs.error=error_info;
        m_param_pub.publish(msgs);
    }
    bool ConnectDevice()
    {
        bool state=false;
        int port;
        std::string ip;
        try
        {
            XmlRpc::XmlRpcValue connect_xml;
            if(m_node.getParam(connect_flag,connect_xml))
            {
                ip=static_cast<std::string>(connect_xml["ip"]);
                port=static_cast<int>(connect_xml["port"]);
                ROS_INFO("ip:%s port:%d\n",ip.c_str(),port);
                state=true;
            }
        }
        catch (ros::Exception &e)
        {
            ROS_ERROR("Error:%s",e.what());
            SendParameterState(connect_flag,false,std::string(e.what()));
            return state;
        }
        if(state)
        {
            if(m_lidar_device && m_lidar_device->Stop())
            {
                m_lidar_device=nullptr;
            }
            m_lidar_device=GetLidarDevice(ip,port);
            if(m_lidar_device)
            {
                SendParameterState(connect_flag,true,"");
            }
            else
            {
                SendParameterState(connect_flag,false,"connect failed!");
            }
        }
        return state;
    }
    bool InitDevice()
    {
        if(!m_lidar_device) return false;
        try
        {
            m_lidar_device->Init();
            SendParameterState(init_device_flag,true,"");
        }
        catch(std::exception &e)
        {
            ROS_ERROR("Error:%s",e.what());
            SendParameterState(init_device_flag,false,std::string(e.what()));
        }
        return true;
    }

    bool DisconnectDevice()
    {
        bool state=true;
        if(m_lidar_device)
        {
            if(m_lidar_device->Stop())
            {
                m_lidar_device=nullptr;
            }
            else
            {
                ROS_ERROR("Error:Failed to stop scanning on the LiDAR sensor.");
            }
        }

        return state;
    }

    bool SetLaserParameter()
    {
    	if(!m_lidar_device) return false;
        bool state=false;
        LaserParameter laserparam;
        try
        {
            XmlRpc::XmlRpcValue laserparam_xml;
            if(m_node.getParam(laser_parameter_flag,laserparam_xml))
            {
                laserparam.level=static_cast<int>(laserparam_xml["level"]);
                laserparam.factor=static_cast<int>(laserparam_xml["factor"]);
                laserparam.pulse_width=static_cast<int>(laserparam_xml["pulse_width"]);
                ROS_INFO("level:%d factor:%d pulse_width:%d.",laserparam.level,laserparam.factor,laserparam.pulse_width);
                state=true;
            }
        }
        catch (ros::Exception &e)
        {
            SendParameterState(laser_parameter_flag,false,std::string(e.what()));
            ROS_ERROR("Error:%s",e.what());
            return state;
        }
        
        try
        {
            m_lidar_device->SetLaser(laserparam);
            SendParameterState(laser_parameter_flag,true,"");
        }
        catch (std::exception &e)
        {
            SendParameterState(laser_parameter_flag,false,std::string(e.what()));
            ROS_ERROR("Error:%s",e.what());
        }
        return state;
    }
    bool SetEchoNumberParameter()
    {
    	if(!m_lidar_device) return false;
        bool state=false;
        int echo_number;
         if(m_node.getParam(echo_number_flag,echo_number))
         {
             ROS_INFO("echo_number:%d.",echo_number);
             state=true;
             try
             {
                 m_lidar_device->SetEchoNumber(echo_number);
                 SendParameterState(echo_number_flag,true,"");
             }
             catch (std::exception &e)
             {
                 SendParameterState(echo_number_flag,false,std::string(e.what()));
                 ROS_ERROR("Error:%s",e.what());
             }
         }
         return state;
    }

    bool SetRawDataType()
    {
    	if(!m_lidar_device) return false;
        bool state=false;
        try
        {
            int32_t type;
            if(m_node.getParam(raw_data_type_flag,type))
            {
                ROS_INFO("raw_data_type:%d.",type);
                state=true;
                m_lidar_device->SetRawDataType((RawDataType)type);
                SendParameterState(raw_data_type_flag,true,"");
            }
        }
        catch (std::exception &e)
        {
            ROS_ERROR("Error:%s",e.what());
            SendParameterState(raw_data_type_flag,false,std::string(e.what()));
        }
        return state;
    }
    bool SetScanMode()
    {
    	if(!m_lidar_device) return false;
        bool state=false;
        try
        {
            int mode;
            if(m_node.getParam(scan_mode_flag,mode))
            {
                ROS_INFO("scan_mode:%d.",mode);
                state=true;
                m_lidar_device->SetScanMode((ScanMode)mode);
                SendParameterState(scan_mode_flag,true,"");
            }
        }
        catch (std::exception &e)
        {
           ROS_ERROR("Error:%s",e.what());
           SendParameterState(scan_mode_flag,false,std::string(e.what()));
        }
        return state;
    }
    bool SetViewParameter()
    {
    	if(!m_lidar_device) return false;
        bool state=false;
        ViewParameter viewparam;
        try
        {
            XmlRpc::XmlRpcValue viewparam_xml;
            if(m_node.getParam(view_parameter_flag,viewparam_xml) && viewparam_xml.getType()==XmlRpc::XmlRpcValue::TypeStruct)
            {
                state=true;
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
                ROS_INFO("frame:%d steps:{%d,%d,%d,%d} perspectives:{%f,%f,%f,%f,%f}",viewparam.frame,viewparam.steps[0],
                        viewparam.steps[1],viewparam.steps[2],viewparam.steps[3],viewparam.perspectives[0],
                        viewparam.perspectives[1],viewparam.perspectives[2],viewparam.perspectives[3],viewparam.perspectives[4]);
            }
        }
        catch (ros::Exception &e)
        {
            SendParameterState(view_parameter_flag,false,std::string(e.what()));
            ROS_ERROR("Error:%s",e.what());
            return state;
        }
        try
        {
            m_lidar_device->SetViewSpeed(viewparam);
            SendParameterState(view_parameter_flag,true,"");
        }
        catch (std::exception &e)
        {
            SendParameterState(view_parameter_flag,false,std::string(e.what()));
            ROS_ERROR("Error:%s",e.what());
        }
        return state;
    }

    bool SetPlayback()
    {
        bool state=false;
        if(m_playback_device && m_playback_device->IsStarted())
            return state;
        std::vector<std::string> files;
        m_node.getParam(playback_flag,files);
        if(files.size())
        {
            state=true;
            m_playback_device=GetPlaybackDevice(files);
            if(m_playback_device)
            {
                m_viewcallback->DisconnectAll();
                m_playback_device->SetParameter(m_dev_param);
                m_playback_device->Init();
                SendParameterState(playback_flag,true,"");
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
            else
            {
                SendParameterState(playback_flag,false,"create playback device failed");
            }
        }
        return state;
    }
    bool StartDevice()
    {
        bool state=false;

        try{
            XmlRpc::XmlRpcValue option_xml;
            if(m_node.getParam(start_device_flag,option_xml))
            {
                state=true;
                bool saveable=static_cast<bool>(option_xml["savable"]);
                int rule=static_cast<int>(option_xml["folder_rule"]);
                std::string path=static_cast<std::string>(option_xml["path"]);
                ROS_INFO("savable:%d folder_rule:%d path:%s.",saveable,rule,path.c_str());
                lidar::WriteRawDataOption option(saveable,(lidar::WriteRawDataOption::FolderRule)rule,path);
                if(m_playback)
                {
                    if(m_playback_device)
                    {
                        if(!m_playback_device->Start(m_viewcallback))
                        {
                            ROS_ERROR("Error:Playback failed.");
                            SendParameterState(start_device_flag,false,"Playback failed.");
                        }
                        else
                        {
                            SendParameterState(start_device_flag,true,"");
                        }
                    }
                }
                else
                {
                    if(m_lidar_device)
                    {
                        if(!m_lidar_device->Start(m_viewcallback,option))
                        {
                            ROS_ERROR("Error:Failed to start scanning on the LiDAR sensor.");
                            SendParameterState(start_device_flag,false,"Failed to start scanning on the LiDAR sensor.");
                        }
                        else
                        {
                            SendParameterState(start_device_flag,true,"");
                        }
                    }
                }
            }
        }
        catch(ros::Exception &e)
        {
            SendParameterState(view_parameter_flag,false,std::string(e.what()));
            ROS_ERROR("Error:%s",e.what());
            return state;
        }
        return state;
    }
    bool PauseDevice()
    {
        if(!m_playback_device) return false;
    	bool state=false;
        int pause=0;
        if(m_node.getParam(pause_device_flag,pause))
        {
            state=true;
            if(m_playback_device && m_playback_device->IsStarted())
            {
                m_playback_device->Pause(bool(pause));
            }
            SendParameterState(pause_device_flag,true,"");
        }
    	return state;
    }
    bool StopDevice()
    {
       bool state=true;
       if(m_playback)
       {
          if(m_playback_device && m_playback_device->IsStarted())
          {
              if(!m_playback_device->Stop())
              {
                   ROS_ERROR("Error:Playback stop failed");
                   SendParameterState(stop_device_flag,false,"Playback stop failed");
              }
              else
              {
                  SendParameterState(stop_device_flag,true,"");
              }
          }
       }
       else
       {
           if(m_lidar_device && m_lidar_device->IsStarted())
           {
               if(!m_lidar_device->Stop())
               {
                   ROS_ERROR("Error:Lidar Device stop failed");
                   SendParameterState(stop_device_flag,false,"Lidar Device stop failed");
               }
               else
               {
                   SendParameterState(stop_device_flag,true,"");
               }
           }
       }
       return state;
    }
    void UpdateParameter(std::string &update_parameter)
    {
        //如此编写，主要是针对多参数设置时，每个参数设置对应
        if(update_parameter.empty())
        {
            m_node.getParam(update_param_flag,update_parameter);
        }

        if(!update_parameter.empty())
        {
            bool state = false;
            int retry = 3;
            do
            {
                if(update_parameter==connect_flag)
                {
                    state=this->ConnectDevice();
                }
                else if(update_parameter==init_device_flag)
                {
                    state=this->InitDevice();
                }
                else if(update_parameter==laser_parameter_flag)
                {
                    state=this->SetLaserParameter();
                }
                else if(update_parameter==echo_number_flag)
                {
                    state=this->SetEchoNumberParameter();
                }
                else if(update_parameter==raw_data_type_flag)
                {
                    state=this->SetRawDataType();
                }
                else if(update_parameter==scan_mode_flag)
                {
                    state=this->SetScanMode();
                }
                else if(update_parameter==playback_flag)
                {
                    state=this->SetPlayback();
                }
                else if(update_parameter==view_parameter_flag)
                {
                    state=this->SetViewParameter();
                }
                else if(update_parameter==disconnect_flag)
                {
                    state=this->DisconnectDevice();
                }
                else if(update_parameter==start_device_flag)
                {
                    state=this->StartDevice();
                }
                else if(update_parameter==pause_device_flag)
                {
                    state=this->PauseDevice();
                }
                else if(update_parameter==stop_device_flag)
                {
                    state=this->StopDevice();
                }
                else if(update_parameter==exit_flag)
                {
                    m_running=false;
                    state=true;
                }
                if (!state)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200)); //200ms
                }
            } while (!state && retry-- > 0);

            ROS_INFO("delete parameter:%s %d %d",update_parameter.c_str(),state,retry);
            m_node.deleteParam(update_parameter);
            //判断update_param参数里数据是否更新为新设置参数,更新就不删除update_param本参数
            std::string update_parameter_temp;
            if(m_node.getParam(update_param_flag,update_parameter_temp))
            {
                if(update_parameter_temp==update_parameter)
                {
                    m_node.deleteParam(update_param_flag);
                }
            }
            update_parameter.clear();
        }
    }
    void ClearParameter()
    {
        if (m_node.hasParam(update_param_flag))
        {
            std::string update_parameter;
            if (m_node.getParam(update_param_flag, update_parameter))
            {
                m_node.deleteParam(update_parameter);
            }
            m_node.deleteParam(update_param_flag);
        }
    }
    void Start()
    {
        m_set_thread=std::thread([this]{
            this->m_running=true;
            ros::Rate loop_rate(10);
            std::string update_parameter;
            while (this->m_running) {
                this->UpdateParameter(update_parameter);
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
