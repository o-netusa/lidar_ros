#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <iostream>
#include <string>
#include <vector>
#include <future>
#include <XmlRpcValue.h>

static std::string update_param="update_device_parameter";
static std::string connect_flag="connect";
static std::string init_device_flag="init_device";
static std::string laser_parameter_flag="laser_parameter";
static std::string echo_number_flag="echo_number";
static std::string raw_data_type_flag="raw_data_type";
static std::string scan_mode_flag="scan_mode";
static std::string playback_flag="playback";
static std::string view_parameter_flag="view_parameter";
static std::string disconnect_flag="disconnect";
static std::string start_device_flag="start_device";
static std::string pause_device_flag="pause_device";
static std::string stop_device_flag="stop_device";

static std::string title =
    "\n*********************************\n"
    "**  a.ConnectDevice(Lidar only)  \n"
    "**  b.InitDevice                \n"
    "**  c.SetLaser                  \n"
    "**  d.SetEchoNumber             \n"
    "**  e.SetRawDataType            \n"
    "**  f.SetScanMode               \n"
    "**  g.SetPlayback               \n"
    "**  h.SetViewSpeed              \n"
    "**  i.Disconnect                \n"
    "**  j.StartDevice           	 \n"
    "**  k.PauseDevice               \n"
    "**  l.StopDevice                \n"
    "** ****press q to quit********* **";

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    ROS_INFO("PointCloud Size:%u\n",msg->points.size());
}

void ClearParameter(ros::NodeHandle &m_node)
    {
       if(m_node.hasParam(update_param))
       {
          std::string update_parameter;
          if(m_node.getParam(update_param,update_parameter))
          {
              m_node.deleteParam(update_parameter);
	   }
          m_node.deleteParam(update_param);
       }
    }

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"test_node");
    ros::NodeHandle node;
    ros::Subscriber cloud_sub=node.subscribe("point_cloud",100,PointCloudCallback);
    ClearParameter(node);
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
                node.setParam(update_param,connect_flag);
                std::cout << "Enter ip-address port: ";
                std::string ip;
                int port;
                std::cin >> ip >> port;
                XmlRpc::XmlRpcValue connect_xml;
                connect_xml["ip"]=ip;
                connect_xml["port"]=port;
                node.setParam(connect_flag,connect_xml);
            }
                break;
            case 'b':
            {
                node.setParam(update_param,init_device_flag);
            }
                break;
            case 'c':
            {
                node.setParam(update_param,laser_parameter_flag);
                int level,factor,pulse_width;
                std::cout << "Enter level[0-19] factor[4-10] pulse_width[0-15]: ";
                std::cin >> level >> factor >> pulse_width;
                XmlRpc::XmlRpcValue Laser_xml;
                Laser_xml["level"]=level;
                Laser_xml["factor"]=factor;
                Laser_xml["pulse_width"]=pulse_width;
                node.setParam(laser_parameter_flag,Laser_xml);
            }
                break;
            case 'd':
            {
                node.setParam(update_param,echo_number_flag);
                std::cout << "Enter echo[1-4]: ";
                int echo;
                std::cin >> echo;
                node.setParam(echo_number_flag,echo);
            }
                break;
            case 'e':
            {
                node.setParam(update_param,raw_data_type_flag);
                std::cout << "Enter raw-data-type[0-1(0 means FPGA, 1 means DSP)]: ";
                int type;
                std::cin >> type;
                node.setParam(raw_data_type_flag,type);
            }
                break;
            case 'f':
            {
                node.setParam(update_param,scan_mode_flag);
                std::cout << "Enter scan-mode[0,1](0 means TWO-WAY, 1 means ONE-WAY): ";
                int mode;
                std::cin >> mode;
                node.setParam(scan_mode_flag,mode);
            }
                break;
            case 'g':
            {
                node.setParam(update_param,playback_flag);
                std::cout << "Enter playback file: ";
                std::string file;
                std::cin >> file;
                std::vector<std::string> file_list;
                file_list.push_back(file);
                node.setParam(playback_flag,file_list);
            }
                break;
            case 'h':
            {
                node.setParam(update_param,view_parameter_flag);
                int frame;
                int steps[4];
                double perspectives[5];
                std::cout << "Enter frame step1 step2 step3 step4 perspective1 perspective2 "
                                                 "perspective3 perspective4 "
                                                 "perspective5: ";
                std::cin >> frame >> steps[0] >> steps[1] >> steps[2] >>steps[3] >> perspectives[0] >> perspectives[1] >>
                        perspectives[2] >> perspectives[3] >> perspectives[4];
                XmlRpc::XmlRpcValue view_xml;
                view_xml["frame"]=frame;
                XmlRpc::XmlRpcValue step_xml;
                int icount=0;
                for(auto step:steps)
                {
                    step_xml[icount++]=step;
                }
                view_xml["steps"]=step_xml;
                icount=0;
                XmlRpc::XmlRpcValue perspective_xml;
                for(auto perspective:perspectives)
                {
                    perspective_xml[icount++]=perspective;
                }
                view_xml["perspectives"]=perspective_xml;
                node.setParam(view_parameter_flag,view_xml);

            }
                break;
            case 'i':
            {
                node.setParam(update_param,disconnect_flag);
            }
                break;
            case 'j':
            {
                node.setParam(update_param,start_device_flag);
                std::cout << "Enter saveable(0,1) folder_rule(0,1,2) save_path: ";
                int saveable;
                int  folder_rule;
                std::string save_path;
                std::cin >> saveable >> folder_rule >> save_path;
                XmlRpc::XmlRpcValue option_xml;
                option_xml["savable"]=saveable;
                option_xml["folder_rule"]=folder_rule;
                option_xml["path"]=save_path;
                node.setParam(start_device_flag,option_xml);
            }
                break;
            case 'k':
            {
                node.setParam(update_param,pause_device_flag);
                std::cout << "Enter pause(0,1)<note:Pause can only be done on the playback>: ";
                int pause;
                node.setParam(pause_device_flag,pause);
            }
            	break;
            case 'l':
            {
                node.setParam(update_param,stop_device_flag);
            }
            	break;
            case 'q':
                flag_loop=false;
                break;
            }

        }
    });
    
    while (flag_loop)
    {
        if(ros::ok()==false)
        {
            flag_loop=false;
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


