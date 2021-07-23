#include "../include/lidar_ros/LidarRosDrive.h"
#include "../include/lidar_ros/ServerRos.h"

#include <map>

namespace onet { namespace lidar_ros {

struct LidarRosDrive::Impl
{

    std::shared_ptr<ServerRos> m_server_ros{nullptr};
    Impl()
    {
        m_server_ros=std::make_shared<ServerRos>();
    }
    bool Init(int argc,char **argv,char *node)
    {
        return m_server_ros->Init(argc,argv,node);
    }
    void Start()
    {
        m_server_ros->Start();
    }
    void Run()
    {
        m_server_ros->Run();
    }
};

LidarRosDrive::LidarRosDrive()
    :m_impl(std::make_shared<LidarRosDrive::Impl>())
{

}

bool LidarRosDrive::Init(int argc,char **argv,char *node)
{
    return m_impl->Init(argc,argv,node);
}

void LidarRosDrive::Start()
{
    m_impl->Start();
}

void LidarRosDrive::Run()
{
    m_impl->Run();
}
}
}

