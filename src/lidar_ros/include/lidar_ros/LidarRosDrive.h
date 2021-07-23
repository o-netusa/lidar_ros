#pragma once
#include <string>
#include <memory>

namespace onet { namespace lidar_ros {

class LidarRosDrive
{

public:
    LidarRosDrive();
    ~LidarRosDrive()=default;
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
