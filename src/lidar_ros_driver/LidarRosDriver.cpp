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
#include <common_msgs/LidarRosService.h>
#include <common_msgs/ParameterMsg.h>
#include <config/DeviceParamsConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>

#include <thread>

#include "ParameterFlag.h"

namespace onet { namespace lidar_ros {

// TODO: find file under config
static auto param_file =
    (fs::path(cppbase::filesystem::GetConfigDir()) / onet::lidar::LIDAR_CHECK_FILE)
        .string();  // default param file

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
    bool m_running{true};
    bool m_playback{false};
    std::string m_update_parameter;
    ros::NodeHandle m_node;        //节点
    ros::Publisher m_cloud_pub;    //点云发布者
    ros::Publisher m_param_pub;    //参数设置状态发布者
    ros::ServiceServer m_service;  // connect参数设置状态

    std::string m_point_cloud_topic_name;
    std::string m_frame_id;
    std::string m_device_ip;
    std::string m_param_file;
    int m_port;

    std::function<void(uint32_t, onet::lidar::PointCloud<onet::lidar::PointXYZI> &)> m_callback{
        nullptr};
    std::function<void(uint32_t, onet::lidar::PointCloud<onet::lidar::PointXYZI> &)> m_callback2{
        nullptr};

    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};
    std::shared_ptr<onet::lidar::DlphDeviceParameter> m_dev_param;
    Impl(ros::NodeHandle node) : m_node(node)
    {
        ClearParameter();
        // TODO: fetch parameters
        m_node.param<std::string>("point_cloud_topic_name", m_point_cloud_topic_name,
                                  "lidar_point_cloud");
        m_node.param<std::string>("device_ip", m_device_ip, "");
        m_node.param<int>("port", m_port);
        m_node.param<std::string>("frame_id", m_frame_id);
        m_node.param<std::string>("lidar_check_parameter_path", m_param_file);

        m_cloud_pub = m_node.advertise<sensor_msgs::PointCloud>(m_point_cloud_topic_name, 100);
        m_param_pub = m_node.advertise<common_msgs::ParameterMsg>(param_msgs, 100);
        onet::lidar::config::Deserialize(m_dev_param, m_param_file);
        m_service = m_node.advertiseService(service_param_flag,
                                            &LidarRosDriver::Impl::HandlerServiceRequest, this);
        m_callback = [this](uint32_t frame_id, lidar::PointCloud<lidar::PointXYZI> &cloud) {
            HandlePointCloud(frame_id, cloud);
        };
        m_callback2 = [this](uint32_t frame_id, lidar::PointCloud<lidar::PointXYZI> &cloud) {
            HandlePointCloud2(frame_id, cloud);
        };
    }
    void HandlePointCloud2(uint32_t frame_id, lidar::PointCloud<onet::lidar::PointXYZI> cloud)
    {
        if (cloud.empty())
        {
            return;
        }
        std::cout << "receive cloud" <<std::endl;
    }

    void HandlePointCloud(uint32_t frame_id, lidar::PointCloud<onet::lidar::PointXYZI> cloud)
    {
        if (cloud.empty())
        {
            return;
        }
        cppbase::Timer<cppbase::us> timer;
        sensor_msgs::PointCloud pointcloud;
        pointcloud.header.stamp = ros::Time::now();
        pointcloud.header.frame_id = m_frame_id;
        pointcloud.points.resize(cloud.size());
        pointcloud.channels.resize(2);
        pointcloud.channels[0].name = "intensities";
        pointcloud.channels[0].values.resize(cloud.size());
        pointcloud.channels[1].name = "rgb";
        pointcloud.channels[1].values.resize(cloud.size());
        for (size_t i = 0; i < cloud.size(); i++)
        {
            const auto &pt = cloud.at(i);
            pointcloud.points[i].x = pt[0];
            pointcloud.points[i].y = pt[1];
            pointcloud.points[i].z = pt[2];
        }
        ROS_INFO("end time:%d us", static_cast<int>(timer.Elapsed()));
        timer.Stop();
        m_cloud_pub.publish(pointcloud);
    }

    void SendParameterState(std::string parameter_flag, bool state, std::string error_info)
    {
        common_msgs::ParameterMsg msgs;
        msgs.parameter_flag = parameter_flag;
        msgs.state = state;
        msgs.error = error_info;
        m_param_pub.publish(msgs);
    }

    // TODO
    void Run()
    {
   
        int port;
        std::string ip;
        
        
        m_lidar_device = GetLidarDevice(ip, port);
        m_lidar_device->Init();

        bool saveable = false;
        int rule = 0;
        std::string path = "/home/Data/";
        onet::lidar::RawDataSavingConfig config(
                    saveable, (lidar::RawDataSavingConfig::FolderRule)rule, path);
                    
        m_lidar_device->SetRawDataSavingConfig(config);
        m_lidar_device->RegisterPointCloudCallback(m_callback2);
        m_lidar_device->Start();
        m_lidar_device->Stop();

    }
    bool HandlerServiceRequest(common_msgs::LidarRosService::Request &req,
                               common_msgs::LidarRosService::Response &res)
    {
        ROS_INFO("Type:%s", req.type.c_str());
        if (req.type == init_device_flag)
        {
            return InitDevice(req, res);
        } else if (req.type == disconnect_flag)
        {
            return DisconnectDevice(req, res);
        } else if (req.type == start_playback_flag)
        {
            return StartPlaybackDevice(req, res);
        } else if (req.type == pause_playback_flag)
        {
            return PausePlaybackDevice(req, res);
        } else if (req.type == stop_device_flag)
        {
            return StopDevice(req, res);
        } else if (req.type == exit_flag)
        {
            if (req.state)
            {
                m_running = false;
                res.success = true;
                res.error = "";
            }
        }
        res.success = true;
        return true;
    }
    bool InitDevice(common_msgs::LidarRosService::Request &req,
                    common_msgs::LidarRosService::Response &res)
    {
        if (!m_lidar_device)
        {
            res.success = false;
            res.error = std::string("sdasda");
            return true;
        }
        if (req.state)
        {
            try
            {
                m_lidar_device->Init();
                res.success = true;
            } catch (std::exception &e)
            {
                ROS_ERROR("Error:%s", e.what());
                res.success = false;
                res.error = std::string(e.what());
            }
        }
        return true;
    }
    bool DisconnectDevice(common_msgs::LidarRosService::Request &req,
                          common_msgs::LidarRosService::Response &res)
    {
        if (req.state)
        {
            if (m_lidar_device)
            {
                if (m_lidar_device->Stop())
                {
                    m_lidar_device = nullptr;
                } else
                {
                    ROS_ERROR("Error:Failed to stop scanning on the LiDAR sensor.");
                }
            }
        }
        res.success = m_lidar_device ? false : true;
        if (!res.success)
        {
            res.error = "Error:Failed to stop scanning on the LiDAR sensor.";
        }
        return true;
    }
    bool StartPlaybackDevice(common_msgs::LidarRosService::Request &req,
                             common_msgs::LidarRosService::Response &res)
    {
        bool state = false;

        try
        {
            if (m_playback && req.state)
            {
                if (m_playback_device)
                {
                    m_playback_device->RegisterPointCloudCallback(m_callback);
                    if (!m_playback_device->Start())
                    {
                        ROS_ERROR("Error:Playback failed.");
                        res.success = false;
                        res.error = "Error:Playback failed.";
                    } else
                    {
                        res.success = true;
                    }
                }
            }
        } catch (ros::Exception &e)
        {
            res.success = false;
            res.error = std::string(e.what());
            ROS_ERROR("Error:%s", e.what());
            return state;
        }
        return state;
    }
    bool PausePlaybackDevice(common_msgs::LidarRosService::Request &req,
                             common_msgs::LidarRosService::Response &res)
    {
        if (!m_playback_device)
            return false;
        bool state = false;
        state = true;
        if (m_playback_device && m_playback_device->IsStarted())
        {
            m_playback_device->Pause(req.state);
            res.success = m_playback_device->IsStarted() ? false : true;
            if (m_playback_device->IsStarted())
            {
                res.error = "Error:Playback pause failed";
            }
        }
        return state;
    }
    bool StopDevice(common_msgs::LidarRosService::Request &req,
                    common_msgs::LidarRosService::Response &res)
    {
        bool state = true;
        if (m_playback)
        {
            if (m_playback_device && m_playback_device->IsStarted())
            {
                if (!m_playback_device->Stop())
                {
                    ROS_ERROR("Error:Playback stop failed");
                }
                res.success = m_playback_device->Stop();
                if (m_playback_device->IsStarted())
                {
                    res.error = "Error:Playback stop failed";
                }
            }
        } else
        {
            if (m_lidar_device && m_lidar_device->IsStarted())
            {
                if (!m_lidar_device->Stop())
                {
                    ROS_ERROR("Error:Lidar Device stop failed");
                }
                res.success = m_lidar_device->Stop();
                if (m_lidar_device->IsStarted())
                {
                    res.error = "Error:Lidar Device stop failed";
                }
            }
        }
        return state;
    }
    bool ConnectDevice()
    {
        bool state = false;
        int port;
        std::string ip;
        try
        {
            XmlRpc::XmlRpcValue connect_xml;
            if (m_node.getParam(connect_flag, connect_xml))
            {
                ip = static_cast<std::string>(connect_xml["ip"]);
                port = static_cast<int>(connect_xml["port"]);
                ROS_INFO("ip:%s port:%d\n", ip.c_str(), port);
                state = true;
            }
        } catch (ros::Exception &e)
        {
            ROS_ERROR("Error:%s", e.what());
            SendParameterState(connect_flag, false, std::string(e.what()));
            return state;
        }
        if (state)
        {
            if (m_lidar_device && m_lidar_device->Stop())
            {
                m_lidar_device = nullptr;
            }
            m_lidar_device = GetLidarDevice(ip, port);
            if (m_lidar_device)
            {
                SendParameterState(connect_flag, true, "");
            } else
            {
                SendParameterState(connect_flag, false, "connect failed!");
            }
        }
        return state;
    }
    bool SetLaserParameter()
    {
        if (!m_lidar_device)
            return false;
        bool state = false;
        LaserParameter laserparam;
        try
        {
            XmlRpc::XmlRpcValue laserparam_xml;
            if (m_node.getParam(laser_parameter_flag, laserparam_xml))
            {
                laserparam.level = static_cast<int>(laserparam_xml["level"]);
                laserparam.factor = static_cast<int>(laserparam_xml["factor"]);
                laserparam.pulse_width = static_cast<int>(laserparam_xml["pulse_width"]);
                ROS_INFO("level:%d factor:%d pulse_width:%d.", laserparam.level, laserparam.factor,
                         laserparam.pulse_width);
                state = true;
            }
        } catch (ros::Exception &e)
        {
            SendParameterState(laser_parameter_flag, false, std::string(e.what()));
            ROS_ERROR("Error:%s", e.what());
            return state;
        }

        try
        {
            m_lidar_device->SetLaser(laserparam);
            SendParameterState(laser_parameter_flag, true, "");
        } catch (std::exception &e)
        {
            SendParameterState(laser_parameter_flag, false, std::string(e.what()));
            ROS_ERROR("Error:%s", e.what());
        }
        return state;
    }
    bool SetEchoNumberParameter()
    {
        if (!m_lidar_device)
            return false;
        bool state = false;
        int echo_number;
        if (m_node.getParam(echo_number_flag, echo_number))
        {
            ROS_INFO("echo_number:%d.", echo_number);
            state = true;
            try
            {
                m_lidar_device->SetEchoNumber(echo_number);
                SendParameterState(echo_number_flag, true, "");
            } catch (std::exception &e)
            {
                SendParameterState(echo_number_flag, false, std::string(e.what()));
                ROS_ERROR("Error:%s", e.what());
            }
        }
        return state;
    }

    bool SetRawDataType()
    {
        if (!m_lidar_device)
            return false;
        bool state = false;
        try
        {
            int32_t type;
            if (m_node.getParam(raw_data_type_flag, type))
            {
                ROS_INFO("raw_data_type:%d.", type);
                state = true;
                m_lidar_device->SetRawDataType((RawDataType)type);
                SendParameterState(raw_data_type_flag, true, "");
            }
        } catch (std::exception &e)
        {
            ROS_ERROR("Error:%s", e.what());
            SendParameterState(raw_data_type_flag, false, std::string(e.what()));
        }
        return state;
    }
    bool SetScanMode()
    {
        if (!m_lidar_device)
            return false;
        bool state = false;
        try
        {
            int mode;
            if (m_node.getParam(scan_mode_flag, mode))
            {
                ROS_INFO("scan_mode:%d.", mode);
                state = true;
                m_lidar_device->SetScanMode((ScanMode)mode);
                SendParameterState(scan_mode_flag, true, "");
            }
        } catch (std::exception &e)
        {
            ROS_ERROR("Error:%s", e.what());
            SendParameterState(scan_mode_flag, false, std::string(e.what()));
        }
        return state;
    }
    bool SetViewParameter()
    {
        if (!m_lidar_device)
            return false;
        bool state = false;
        ViewParameter viewparam;
        try
        {
            XmlRpc::XmlRpcValue viewparam_xml;
            if (m_node.getParam(view_parameter_flag, viewparam_xml) &&
                viewparam_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                state = true;
                viewparam.frame = static_cast<int>(viewparam_xml["frame"]);
                viewparam.steps[0] = static_cast<int>(viewparam_xml["steps"][0]);
                viewparam.steps[1] = static_cast<int>(viewparam_xml["steps"][1]);
                viewparam.steps[2] = static_cast<int>(viewparam_xml["steps"][2]);
                viewparam.steps[3] = static_cast<int>(viewparam_xml["steps"][3]);
                viewparam.perspectives[0] = static_cast<double>(viewparam_xml["perspectives"][0]);
                viewparam.perspectives[1] = static_cast<double>(viewparam_xml["perspectives"][1]);
                viewparam.perspectives[2] = static_cast<double>(viewparam_xml["perspectives"][2]);
                viewparam.perspectives[3] = static_cast<double>(viewparam_xml["perspectives"][3]);
                viewparam.perspectives[4] = static_cast<double>(viewparam_xml["perspectives"][4]);
                ROS_INFO("frame:%d steps:{%d,%d,%d,%d} perspectives:{%f,%f,%f,%f,%f}",
                         viewparam.frame, viewparam.steps[0], viewparam.steps[1],
                         viewparam.steps[2], viewparam.steps[3], viewparam.perspectives[0],
                         viewparam.perspectives[1], viewparam.perspectives[2],
                         viewparam.perspectives[3], viewparam.perspectives[4]);
            }
        } catch (ros::Exception &e)
        {
            SendParameterState(view_parameter_flag, false, std::string(e.what()));
            ROS_ERROR("Error:%s", e.what());
            return state;
        }
        try
        {
            m_lidar_device->SetViewSpeed(viewparam);
            SendParameterState(view_parameter_flag, true, "");
        } catch (std::exception &e)
        {
            SendParameterState(view_parameter_flag, false, std::string(e.what()));
            ROS_ERROR("Error:%s", e.what());
        }
        return state;
    }

    bool SetPlayback()
    {
        bool state = false;
        if (m_playback_device && m_playback_device->IsStarted())
            return state;
        std::vector<std::string> files;
        m_node.getParam(playback_flag, files);
        if (files.size())
        {
            state = true;
            m_playback_device = GetPlaybackDevice(files);
            if (m_playback_device)
            {
                m_playback_device->SetParameter(m_dev_param);
                m_playback_device->Init();
                SendParameterState(playback_flag, true, "");
                m_playback = true;
            } else
            {
                SendParameterState(playback_flag, false, "create playback device failed");
            }
        }
        return state;
    }
    bool StartDevice()
    {
        bool state = false;

        try
        {
            XmlRpc::XmlRpcValue option_xml;
            if (m_node.getParam(start_device_flag, option_xml))
            {
                state = true;
                bool saveable = static_cast<bool>(option_xml["savable"]);
                int rule = static_cast<int>(option_xml["folder_rule"]);
                std::string path = static_cast<std::string>(option_xml["path"]);
                ROS_INFO("savable:%d folder_rule:%d path:%s.", saveable, rule, path.c_str());
                onet::lidar::RawDataSavingConfig config(
                    saveable, (lidar::RawDataSavingConfig::FolderRule)rule, path);

                if (!m_playback)
                {
                    if (m_lidar_device)
                    {
                        m_lidar_device->SetRawDataSavingConfig(config);
                        m_lidar_device->RegisterPointCloudCallback(m_callback);
                        if (!m_lidar_device->Start())
                        {
                            ROS_ERROR("Error:Failed to start scanning on the LiDAR sensor.");
                            SendParameterState(start_device_flag, false,
                                               "Failed to start scanning on the LiDAR sensor.");
                        } else
                        {
                            SendParameterState(start_device_flag, true, "");
                        }
                    }
                }
            }
        } catch (ros::Exception &e)
        {
            SendParameterState(view_parameter_flag, false, std::string(e.what()));
            ROS_ERROR("Error:%s", e.what());
            return state;
        }
        return state;
    }

    void UpdateParameter()
    {
        //如此编写，主要是针对多参数设置时，每个参数设置对应
        if (m_update_parameter.empty())
        {
            m_node.getParam(update_param_flag, m_update_parameter);
        }

        if (!m_update_parameter.empty())
        {
            bool state = false;
            int retry = 3;
            do
            {
                if (m_update_parameter == connect_flag)
                {
                    state = this->ConnectDevice();
                } else if (m_update_parameter == laser_parameter_flag)
                {
                    state = this->SetLaserParameter();
                } else if (m_update_parameter == echo_number_flag)
                {
                    state = this->SetEchoNumberParameter();
                } else if (m_update_parameter == raw_data_type_flag)
                {
                    state = this->SetRawDataType();
                } else if (m_update_parameter == scan_mode_flag)
                {
                    state = this->SetScanMode();
                } else if (m_update_parameter == playback_flag)
                {
                    state = this->SetPlayback();
                } else if (m_update_parameter == view_parameter_flag)
                {
                    state = this->SetViewParameter();
                } else if (m_update_parameter == start_device_flag)
                {
                    state = StartDevice();
                }
                if (!state)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 200ms
                }
            } while (!state && retry-- > 0);

            ROS_INFO("delete parameter:%s %d %d", m_update_parameter.c_str(), state, retry);
            m_node.deleteParam(m_update_parameter);
            //判断update_param参数里数据是否更新为新设置参数,更新就不删除update_param本参数
            std::string update_parameter_temp;
            if (m_node.getParam(update_param_flag, update_parameter_temp))
            {
                if (update_parameter_temp == m_update_parameter)
                {
                    m_node.deleteParam(update_param_flag);
                }
            }
            m_update_parameter.clear();
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
};

LidarRosDriver::LidarRosDriver(ros::NodeHandle node)
    : m_impl(std::make_shared<LidarRosDriver::Impl>(node))
{}

void LidarRosDriver::UpdateParameter()
{
    m_impl->UpdateParameter();
}

bool LidarRosDriver::IsRunning() const
{
    return m_impl->m_running;
}

void LidarRosDriver::Run()
{
    m_impl->Run();
}

}}  // namespace onet::lidar_ros
