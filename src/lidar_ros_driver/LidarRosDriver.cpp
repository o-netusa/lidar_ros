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
#include <pcl_conversions/pcl_conversions.h>
#include <processing/PointCloudProcessing.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <thread>

#include "ParameterFlag.h"

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
    bool m_running{true};
    bool m_auto_start{true};
    bool m_save_bag{false};
    std::string m_update_parameter;
    rosbag::Bag m_bag;
    ros::NodeHandle m_node;        //节点
    ros::Publisher m_cloud_pub;    //点云发布者
    ros::Publisher m_param_pub;    //参数设置状态发布者
    ros::ServiceServer m_service;  // connect参数设置状态

    std::string m_point_cloud_topic_name{"lidar_point_cloud"};
    std::string m_frame_id{"lidar"};
    std::string m_device_ip{"192.168.1.2"};
    int m_port{2368};
    std::string m_playback_file_path;
    int m_playback_fps{10};

    int near_noise_dist{0};
    int near_noise_intensity{0};
    int time_dif{0};
    int high_pul{0};
    int time_fly{0};
    int pulse_dif{0};
    int gather_size{0};

    bool m_enable_remove_noise = true;
    float m_noise_distance_threshold = 1.0;
    float m_noise_area_sq = 0.5;
    float m_noise_range = 500.0;

    std::string m_galvanometer_file{""};
    int m_frame{10};

    std::function<void(uint32_t, onet::lidar::PointCloud<onet::lidar::PointXYZI> &)> m_callback{
        nullptr};

    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};
    std::shared_ptr<onet::lidar::DlphDeviceParameter> m_dev_param;

    void InitLidar(ros::NodeHandle node)
    {
        near_noise_dist =
            node.param<int>("/onet_lidar_ros_driver/near_noise_dist", near_noise_dist);
        near_noise_intensity =
            node.param<int>("/onet_lidar_ros_driver/near_noise_intensity", near_noise_intensity);
        time_dif = node.param<int>("/onet_lidar_ros_driver/time_dif", time_dif);
        high_pul = node.param<int>("/onet_lidar_ros_driver/high_pul", high_pul);
        time_fly = node.param<int>("/onet_lidar_ros_driver/time_fly", time_fly);
        pulse_dif = node.param<int>("/onet_lidar_ros_driver/pulse_dif", pulse_dif);
        gather_size = node.param<int>("/onet_lidar_ros_driver/gather_size", gather_size);

        m_auto_start = m_node.param<bool>("/onet_lidar_ros_driver/auto_start", m_auto_start);
        m_save_bag = m_node.param<bool>("/onet_lidar_ros_driver/save_bag", m_save_bag);
        m_point_cloud_topic_name = m_node.param<std::string>(
            "/onet_lidar_ros_driver/point_cloud_topic_name", m_point_cloud_topic_name);
        m_device_ip = m_node.param<std::string>("/onet_lidar_ros_driver/device_ip", m_device_ip);
        m_port = m_node.param<int>("/onet_lidar_ros_driver/port", m_port);
        m_frame_id = m_node.param<std::string>("/onet_lidar_ros_driver/frame_id", m_frame_id);
        m_playback_file_path = m_node.param<std::string>(
            "/onet_lidar_ros_driver/playback_file_path", m_playback_file_path);

        m_enable_remove_noise =
            m_node.param<bool>("/onet_lidar_ros_driver/enable_remove_noise", m_enable_remove_noise);
        m_noise_distance_threshold = m_node.param<float>(
            "/onet_lidar_ros_driver/noise_distance_threshold", m_noise_distance_threshold);
        m_noise_area_sq =
            m_node.param<float>("/onet_lidar_ros_driver/noise_area_sq", m_noise_area_sq);
        m_noise_range = m_node.param<float>("/onet_lidar_ros_driver/noise_range", m_noise_range);

        m_galvanometer_file = m_node.param<std::string>("/onet_lidar_ros_driver/galvanometer_file",
                                                        m_galvanometer_file);
        m_frame = m_node.param<int>("/onet_lidar_ros_driver/frame_rate", m_frame);
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
        m_param_pub = m_node.advertise<common_msgs::ParameterMsg>(param_msgs, 100);
        m_service = m_node.advertiseService(service_param_flag,
                                            &LidarRosDriver::Impl::HandlerServiceRequest, this);
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

        ROS_INFO("end time:%d us", static_cast<int>(timer.Elapsed()));
        timer.Stop();
        m_cloud_pub.publish(msg_pointcloud);
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

        onet::lidar::processing::SetRemovedNoisePoints(m_enable_remove_noise);
        onet::lidar::processing::SetNoiseDistThreshold(m_noise_distance_threshold);
        onet::lidar::processing::SetNoiseAreaSq(m_noise_area_sq);
        onet::lidar::processing::SetNoiseRange(m_noise_range);
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
                m_playback_device->SetFPS(m_playback_fps);
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
                // apply even galvanometer if there is a txt file exist
                if (!m_galvanometer_file.empty())
                {
                    auto load_galvanometer = [this](std::vector<uint16_t> &data,
                                                    const std::string &filename) {
                        std::ifstream file(filename);
                        while (true)
                        {
                            std::string text;
                            if (std::getline(file, text, '\n'))
                            {
                                uint16_t value = atoi(text.c_str());
                                if (value >= 0 && value <= 4095)
                                    data.push_back(atoi(text.c_str()));
                            } else
                            {
                                ROS_INFO("Finished reading information from galvanometer.");
                                break;
                            }
                        }
                    };

                    std::vector<uint16_t> galvanometer_param;
                    load_galvanometer(galvanometer_param, m_galvanometer_file);
                    if (galvanometer_param.empty())
                    {
                        ROS_ERROR(
                            "Error: Failed to load galvanometer file, please check the path of the "
                            "galvanometer file!");
                    }
                    std::sort(galvanometer_param.begin(), galvanometer_param.end());

                 LidarParameter lidar_param = m_lidar_device->GetLidarParameter();
                {
                    lidar::RegisterData close_laser_param;
                    close_laser_param.parameters[0] = 0; //enable
                    close_laser_param.parameters[1] = 0; //triger mode default 0
                    close_laser_param.parameters[2] = lidar_param.laser.factor;  //laser factor [1-10]
                    close_laser_param.parameters[3] = lidar_param.laser.level;; // laser power []
                    close_laser_param.parameters[4] = lidar_param.laser.pulse_width;   
                    m_lidar_device->SetRegisterParameter(lidar::LASER_CTL, close_laser_param);
                }
                  sleep(1);
                {
                    lidar::RegisterData open_laser_param;
                    open_laser_param.parameters[0] = 1;
                    open_laser_param.parameters[1] = 0;
                    open_laser_param.parameters[2] = lidar_param.laser.factor;
                    open_laser_param.parameters[3] = lidar_param.laser.level;
                    open_laser_param.parameters[4] = lidar_param.laser.pulse_width;
                    m_lidar_device->SetRegisterParameter(lidar::LASER_CTL, open_laser_param);
                }
                sleep(1);
                m_lidar_device->SetLaser(lidar_param.laser);
                {
                    lidar::RegisterData TimeWin;
                    TimeWin.parameters[0] = lidar_param.min_time;
                    TimeWin.parameters[1] = lidar_param.max_time; 
                    ROS_INFO("max %d",TimeWin.parameters[0]);
                    ROS_INFO("min %d",TimeWin.parameters[1]);
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_TIME_RANGE, TimeWin);
                }
                {
                    //删除近处杂点
                    lidar::RegisterData data;
                    data.parameters[0] = near_noise_dist;
                    data.parameters[1] = near_noise_intensity;
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_REG6, data);
                }
                {
                    //删除远处重影
                    lidar::RegisterData data;
                    data.parameters[0] = pulse_dif;
                    data.parameters[1] = time_fly;
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_REG5, data);
                }
                {
                    lidar::RegisterData data;
                    data.parameters[0] = high_pul;
                    data.parameters[1] = time_dif;
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_REG4, data);
                }


                    try
                    {
                        m_lidar_device->SetGalvanometerParameter(m_frame, galvanometer_param);
                          ROS_INFO("m_frame %d",m_frame);
                       
                    } catch (const std::exception &e)
                    {
                        ROS_ERROR("Error: the galvanometer parameter is illegal!");
                    }
                }

              
                {
                    //设置采样频率
                    lidar::RegisterData data;
                    data.parameters[0] = gather_size;
                     ROS_INFO("gathersize  %d", data.parameters[0]);
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_REG0, data);
                }
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

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Service related functions
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void SendParameterState(std::string parameter_flag, bool state, std::string error_info)
    {
        common_msgs::ParameterMsg msgs;
        msgs.parameter_flag = parameter_flag;
        msgs.state = state;
        msgs.error = error_info;
        m_param_pub.publish(msgs);
    }

    bool HandlerServiceRequest(common_msgs::LidarRosService::Request &req,
                               common_msgs::LidarRosService::Response &res)
    {
        ROS_INFO("Type: %s", req.type.c_str());
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
            if (m_playback_device && req.state)
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
