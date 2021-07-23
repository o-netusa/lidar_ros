#pragma once
#include <memory>
namespace onet {
namespace lidar_ros {
class ServerRos
{
public:
    ServerRos();
    ~ServerRos();
public:
    bool Init(int argc,char **argv,char *node);
    void Start();
    void Run();
private:
    struct Impl;
    std::shared_ptr<Impl> m_impl;
};
}
}
