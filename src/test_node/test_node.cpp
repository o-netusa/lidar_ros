#include <ParameterFlag.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <common_msgs/ParameterMsg.h>
#include <common_msgs/LidarRosService.h>

#include <future>
#include <iostream>
#include <string>
#include <vector>

static std::string title =
    "\n*********************************\n"
    "**  a.ConnectDevice(Lidar only) \n"
    "**  b.InitDevice                \n"
    "**  c.SetLaser                  \n"
    "**  d.SetEchoNumber             \n"
    "**  e.SetRawDataType            \n"
    "**  f.SetScanMode               \n"
    "**  g.SetPlayback               \n"
    "**  h.SetViewSpeed              \n"
    "**  i.Disconnect                \n"
    "**  j.StartDevice           	 \n"
    "**  k.StartPlayback           	 \n"
    "**  l.PauseDevice               \n"
    "**  m.StopDevice                \n"
    "** ****press q to quit********* **";

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    ROS_INFO("PointCloud Size:%d\n", static_cast<int>(msg->points.size()));
}
void ParamCallback(const common_msgs::ParameterMsg::ConstPtr &msg)
{
    ROS_INFO("Parameter type:%s success:%d error:%s\n", msg->parameter_flag.c_str(), msg->state,
             msg->error.c_str());
}

void ClearParameter(ros::NodeHandle &m_node)
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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle node;
    ClearParameter(node);
    ros::Subscriber cloud_sub = node.subscribe(pointcloud_msgs, 100, PointCloudCallback);
    ros::Subscriber param_sub = node.subscribe(param_msgs, 100, ParamCallback);
    ros::service::waitForService(service_param_flag);
    ros::ServiceClient lidar_ros_driver_client =
        node.serviceClient<common_msgs::LidarRosService>(service_param_flag);
    ros::Rate loop_rate(1);
    std::atomic_bool flag_loop{true};
    auto op_future = std::async(std::launch::async, [&] {
        while (flag_loop)
        {
            char op;
            std::cout << title << std::endl;
            std::cout << "Enter key: ";
            std::cin >> op;
            switch (op)
            {
                case 'a':
                {
                    std::cout << "Enter ip-address port: ";
                    std::string ip;
                    int port;
                    std::cin >> ip >> port;
                    XmlRpc::XmlRpcValue connect_xml;
                    connect_xml["ip"] = ip;
                    connect_xml["port"] = port;
                    node.setParam(update_param_flag, connect_flag);
                    node.setParam(connect_flag, connect_xml);
                }
                break;
                case 'b':
                {
                    common_msgs::LidarRosService service_param;
                    service_param.request.type = init_device_flag;
                    service_param.request.state = true;
                    bool state = false;
                    while (!state)
                    {
                        state = lidar_ros_driver_client.call(service_param);
                    }
                    if (state)
                    {
                        if (service_param.response.success)
                        {
                            ROS_INFO("%s:%d.", init_device_flag.c_str(),
                                     service_param.response.success);
                        } else
                        {
                            ROS_INFO("%s", service_param.response.error.c_str());
                        }
                    } else
                    {
                        ROS_INFO("call Failed %s:%d.", init_device_flag.c_str(),
                                 service_param.response.success);
                    }
                }
                break;
                case 'c':
                {
                    int level, factor, pulse_width;
                    std::cout << "Enter level[0-19] factor[4-10] pulse_width[0-15]: ";
                    std::cin >> level >> factor >> pulse_width;
                    XmlRpc::XmlRpcValue Laser_xml;
                    Laser_xml["level"] = level;
                    Laser_xml["factor"] = factor;
                    Laser_xml["pulse_width"] = pulse_width;
                    node.setParam(update_param_flag, laser_parameter_flag);
                    node.setParam(laser_parameter_flag, Laser_xml);
                }
                break;
                case 'd':
                {
                    std::cout << "Enter echo[1-4]: ";
                    int echo;
                    std::cin >> echo;
                    node.setParam(update_param_flag, echo_number_flag);
                    node.setParam(echo_number_flag, echo);
                }
                break;
                case 'e':
                {
                    std::cout << "Enter raw-data-type[0-1(0 means FPGA, 1 means DSP)]: ";
                    int type;
                    std::cin >> type;
                    node.setParam(update_param_flag, raw_data_type_flag);
                    node.setParam(raw_data_type_flag, type);
                }
                break;
                case 'f':
                {
                    std::cout << "Enter scan-mode[0,1](0 means TWO-WAY, 1 means ONE-WAY): ";
                    int mode;
                    std::cin >> mode;
                    node.setParam(update_param_flag, scan_mode_flag);
                    node.setParam(scan_mode_flag, mode);
                }
                break;
                case 'g':
                {
                    std::cout << "Enter playback file number(number>0): ";
                    int count;
                    std::cin >> count;
                    std::vector<std::string> file_list;
                    for (int i = 0; i < count; i++)
                    {
                        std::cout << "Enter the " << i + 1 << " playback file:";
                        std::string file;
                        std::cin >> file;

                        file_list.push_back(file);
                    }
                    node.setParam(update_param_flag, playback_flag);
                    node.setParam(playback_flag, file_list);
                }
                break;
                case 'h':
                {
                    int frame;
                    int steps[4];
                    double perspectives[5];
                    std::cout << "Enter frame step1 step2 step3 step4 perspective1 perspective2 "
                                 "perspective3 perspective4 "
                                 "perspective5: ";
                    std::cin >> frame >> steps[0] >> steps[1] >> steps[2] >> steps[3] >>
                        perspectives[0] >> perspectives[1] >> perspectives[2] >> perspectives[3] >>
                        perspectives[4];
                    XmlRpc::XmlRpcValue view_xml;
                    view_xml["frame"] = frame;
                    XmlRpc::XmlRpcValue step_xml;
                    int icount = 0;
                    for (auto step : steps)
                    {
                        step_xml[icount++] = step;
                    }
                    view_xml["steps"] = step_xml;
                    icount = 0;
                    XmlRpc::XmlRpcValue perspective_xml;
                    for (auto perspective : perspectives)
                    {
                        perspective_xml[icount++] = perspective;
                    }
                    view_xml["perspectives"] = perspective_xml;
                    node.setParam(update_param_flag, view_parameter_flag);
                    node.setParam(view_parameter_flag, view_xml);
                }
                break;
                case 'i':
                {
                    common_msgs::LidarRosService service_param;
                    service_param.request.type = disconnect_flag;
                    // service_param.request.type=1;
                    service_param.request.state = true;
                    if (lidar_ros_driver_client.call(service_param))
                    {
                        if (service_param.response.success)
                        {
                            ROS_INFO("%s:%d.", disconnect_flag.c_str(),
                                     service_param.response.success);
                        } else
                        {
                            ROS_INFO("%s", service_param.response.error.c_str());
                        }
                    }
                }
                break;
                case 'j':
                {
                    std::cout << "Enter saveable(0,1) folder_rule(0,1,2) save_path: ";
                    bool saveable;
                    int folder_rule;
                    std::string save_path;
                    std::cin >> saveable >> folder_rule >> save_path;
                    XmlRpc::XmlRpcValue option_xml;
                    option_xml["savable"] = saveable;
                    option_xml["folder_rule"] = folder_rule;
                    option_xml["path"] = save_path;
                    node.setParam(update_param_flag, start_device_flag);
                    node.setParam(start_device_flag, option_xml);
                }
                break;

                case 'k':
                {
                    common_msgs::LidarRosService service_param;
                    service_param.request.type = start_playback_flag;
                    // service_param.request.type=2;
                    service_param.request.state = true;
                    if (lidar_ros_driver_client.call(service_param))
                    {
                        if (service_param.response.success)
                        {
                            ROS_INFO("%s:%d.", start_playback_flag.c_str(),
                                     service_param.response.success);
                        } else
                        {
                            ROS_INFO("%s", service_param.response.error.c_str());
                        }
                    }
                }
                break;
                case 'l':
                {
                    std::cout << "Enter pause(0,1): ";
                    common_msgs::LidarRosService service_param;
                    service_param.request.type = pause_playback_flag;
                    // service_param.request.type=3;
                    std::cin >> service_param.request.state;
                    if (lidar_ros_driver_client.call(service_param))
                    {
                        if (service_param.response.success)
                        {
                            ROS_INFO("%s:%d.", pause_playback_flag.c_str(),
                                     service_param.response.success);
                        } else
                        {
                            ROS_INFO("%s", service_param.response.error.c_str());
                        }
                    }
                }
                break;
                case 'm':
                {
                    common_msgs::LidarRosService service_param;
                    service_param.request.type = stop_device_flag;
                    // service_param.request.type=4;
                    service_param.request.state = true;
                    if (lidar_ros_driver_client.call(service_param))
                    {
                        if (service_param.response.success)
                        {
                            ROS_INFO("%s:%d.", stop_device_flag.c_str(),
                                     service_param.response.success);
                        } else
                        {
                            ROS_INFO("%s", service_param.response.error.c_str());
                        }
                    }
                }
                break;
                case 'q':
                {
                    common_msgs::LidarRosService service_param;
                    service_param.request.type = exit_flag;
                    // service_param.request.type=5;
                    service_param.request.state = true;
                    if (lidar_ros_driver_client.call(service_param))
                    {
                        if (service_param.response.success)
                        {
                            ROS_INFO("%s:%d.", exit_flag.c_str(), service_param.response.success);
                        } else
                        {
                            ROS_INFO("%s", service_param.response.error.c_str());
                        }
                    }
                    flag_loop = false;
                }
                break;
            }
        }
    });

    while (flag_loop)
    {
        if (ros::ok() == false)
        {
            flag_loop = false;
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
