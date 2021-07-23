#include "../include/lidar_ros/ServerRos.h"
#include "../include/lidar_ros/types.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <common_msg/network.h>
#include <common_msg/laserparameter.h>
#include <common_msg/echonumber.h>
#include <common_msg/rawdatatype.h>
#include <common_msg/scanmode.h>
#include <common_msg/viewparameter.h>
#include <common_msg/playback.h>
#include <common_msg/gatharparameter.h>
#include <common_msg/Register_msg.h>
#include <thread>

#include <LidarDevice.h>
#include <PlaybackDevice.h>
#include <DeviceManager.h>
#include <Types.h>

namespace onet {
namespace lidar_ros {

class ViewerCallback//:public onet::lidar::DeviceCallback
{
public:
    void SetVisualizer()
    {

    }
    //void HandlePointCloud(uint32_t frame_id, std::shared_ptr<PointCloud> cloud,[[maybe_unused]] const std::string &file_name = {})
    //{
    //    if (!cloud)
    //    {
    //        return;
    //    }
    //}
    void PlaybackDone()
    {

    }
    ros::Publisher m_cloud_pub;             //点云发布者
};

onet::lidar::PlaybackDevice* GetPlaybackDevice(const std::vector<std::string>& file_list)
{
    static uuids::uuid play_device_id=uuids::uuid();
    if(!play_device_id.is_nil())
    {
        onet::lidar::DeviceManager::GetInstance().RemoveDevice(play_device_id);
        play_device_id=uuids::uuid();
    }
    play_device_id=lidar::DeviceManager::GetInstance().CreateDevice(file_list);
    return dynamic_cast<lidar::PlaybackDevice*>(lidar::DeviceManager::GetInstance().GetDevice(play_device_id));
}

onet::lidar::LidarDevice* GetLidarDevice(const std::string &strIP,int port)
{
    static uuids::uuid lidar_device_id=uuids::uuid();
    if(lidar_device_id.is_nil())
    {
        onet::lidar::DeviceManager::GetInstance().RemoveDevice(lidar_device_id);
        lidar_device_id=uuids::uuid();
    }
    lidar_device_id=onet::lidar::DeviceManager::GetInstance().CreateDevice(strIP,port);
    return dynamic_cast<lidar::LidarDevice*>(lidar::DeviceManager::GetInstance().GetDevice(lidar_device_id));
}

struct ServerRos::Impl
{
    bool m_start{false};
    std::thread m_thread;
    ros::NodeHandle m_node;                 //节点
    ros::Publisher m_test_pub;              //测试发布者
    ros::Subscriber m_test_sub;             //测试订阅者
    ros::Subscriber m_register_sub;         //注册订阅者
    ros::Subscriber m_network_sub;          //Network订阅者
    ros::Subscriber m_laserparameter_sub;   //LaserParameter订阅者
    ros::Subscriber m_echonumber_sub;       //EchoNumber订阅者
    ros::Subscriber m_rawdatatype_sub;      //RawDataType订阅者
    ros::Subscriber m_scanmode_sub;         //ScanMode订阅者
    ros::Subscriber m_viewparameter_sub;    //ViewParameter订阅者
    ros::Subscriber m_playback_sub;         //Playback订阅者
    ros::Subscriber m_gatharparameter_sub;  //GatherParameter订阅者
    ViewerCallback              *m_viewcallback;
    lidar::LidarDevice          *m_lidar_device{nullptr};
    lidar::PlaybackDevice       *m_playback_device{nullptr};

    bool Init(int argc,char **argv,char *node)
    {
        ros::init(argc,argv,node);
        return ros::master::check();
    }
    void Start()
    {
        ros::start();
        m_test_sub=m_node.subscribe("test_sub",100,&ServerRos::Impl::testCallback,this);
        m_test_pub=m_node.advertise<std_msgs::String>("test",100);
        m_register_sub=m_node.subscribe("Register",100,&ServerRos::Impl::RegisterTopic,this);
    }
    void Run()
    {
        ros::spin();
    }

    void testCallback(const std_msgs::String::ConstPtr& msg)
    {

    }

    void RegisterTopic(const common_msg::Register_msg::ConstPtr &msg)
    {
        //bool RegisterTopic(emTopic type,const std::string &topic);
        ROS_INFO("ServerRos Register type=%d topic name=%s",msg->type,msg->topicname.c_str());
        emTopic type=(emTopic)msg->type;
        switch (type)
        {
         case emTopic::NetWork:
        {
            m_network_sub=m_node.subscribe(msg->topicname,100,&ServerRos::Impl::networkCallback,this);
        }
            break;
        case emTopic::EchoNumber:
        {
            m_echonumber_sub=m_node.subscribe(msg->topicname,100,&ServerRos::Impl::echoNumberCallback,this);
        }
            break;
        case emTopic::LaserParameter:
        {
            m_laserparameter_sub=m_node.subscribe(msg->topicname,100,&ServerRos::Impl::laserparameterCallback,this);
        }
            break;
        case emTopic::RawDataType:
        {
            m_rawdatatype_sub=m_node.subscribe(msg->topicname,100,&ServerRos::Impl::rawdatatypeCallback,this);
        }
            break;
        case emTopic::ScanMode:
        {
            m_scanmode_sub=m_node.subscribe(msg->topicname,100,&ServerRos::Impl::scanmodeCallback,this);
        }
            break;
        case emTopic::ViewParameter:
        {
            m_viewparameter_sub=m_node.subscribe(msg->topicname,100,&ServerRos::Impl::viewparameterCallback,this);
        }
            break;
        case emTopic::PlayBack:
        {
            m_playback_sub=m_node.subscribe(msg->topicname,100,&ServerRos::Impl::playbackCallback,this);
        }
            break;
        case emTopic::GatharParameter:
        {
            m_gatharparameter_sub=m_node.subscribe(msg->topicname,100,&ServerRos::Impl::gatharparameterCallback,this);
        }
            break;
        case emTopic::Sensor:
        {
            if(m_viewcallback)
            {
                m_viewcallback->m_cloud_pub=m_node.advetise<sensor_msgs::PointCloud>(msg->topicname,100);
            }
        }
            break;
        }
    }
    void networkCallback(const common_msg::network::ConstPtr& msg)
    {
        if(msg->open)
        {
            if(!m_lidar_device)
            {
                m_lidar_device=GetLidarDevice(msg->ip,msg->port);
                if(m_lidar_device)
                {
                    if(m_lidar_device->Init())
                    {
                        ROS_INFO("Succed to connect to LiDAR Sensor.");
                    }
                    else
                    {
                        ROS_INFO("Failed to connect to LiDAR Sensor.");
                        m_lidar_device=nullptr;
                    }
                }
            }
        }
        else
        {
            if(m_lidar_device)
            {
                if(m_lidar_device->Stop())
                {
                    m_lidar_device=nullptr;
                }
                else
                {
                    ROS_INFO("Failed to stop scanning on the LiDAR sensor.");
                }
            }
        }
    }

    void laserparameterCallback(const common_msg::laserparameter::ConstPtr& msg)
    {
        if(m_lidar_device)
         {
             LaserParameter laserparam;
             //TODO verify the factor's range
             laserparam.factor=msg->factor;
             laserparam.level=msg->level;
             laserparam.pulse_width=msg->pulse_width;
             try
             {
                 m_lidar_device->SetLaser(laserparam);
             }
             catch (std::exception &e)
             {
                 ROS_INFO("Error:%s",e.what());
             }
         }
    }

    void echoNumberCallback(const common_msg::echonumber::ConstPtr& msg)
    {
        if(m_lidar_device)
        {
            int32_t echo_number=msg->echonumber;
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

    void rawdatatypeCallback(const common_msg::rawdatatype::ConstPtr& msg)
    {
        try
        {
            m_lidar_device->SetRawDataType(static_cast<onet::lidar::RawDataType>(msg->type));
        }
        catch (std::exception &e)
        {
            ROS_INFO("Error:%s",e.what());
        }
    }

    void scanmodeCallback(const common_msg::scanmode::ConstPtr& msg)
    {
        if(m_lidar_device)
        {
            ScanMode mode = (ScanMode)msg->mode;
            try
            {
                m_lidar_device->SetScanMode(mode);
            }
            catch (std::exception &e)
            {
               ROS_INFO("Error:%s",e.what());
            }
        }
    }

    void viewparameterCallback(const common_msg::viewparameter::ConstPtr& msg)
    {
        if(m_lidar_device)
        {
            ViewParameter viewparam;
            viewparam.frame=msg->frame
            for(int i=三0;i<4;i++)
            {
                viewparam.steps[i] = msg->steps[i];
            }
            for(int i=0;i<4;i++)
            {
                viewparam.perspectives[i]=msg->perspectives[i];
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
    }

    void playbackCallback(const common_msg::playback::ConstPtr& msg)
    {
        if(!m_playback_device)
        {
            std::vector<std::string> files;
            for(auto file:msg->files)
            {
                files.push_back(file);
            }
            if(files.size())
            {
                m_playback_device=GetPlaybackDevice(files);
                if(m_playback_device)
                {

                }
                else
                {

                }
            }
        }
    }

    void gatharparameterCallback(const common_msg::gatharparameter::ConstPtr& msg)
    {

    }
};

ServerRos::ServerRos()
    :m_impl(std::make_shared<ServerRos::Impl>())
{

}

ServerRos::~ServerRos()
{

}

bool ServerRos::Init(int argc,char **argv,char *node)
{
    return m_impl->Init(argc,argv,node);
}

void ServerRos::Start()
{

}

void ServerRos::Run()
{

}

}
}
