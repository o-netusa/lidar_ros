#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <iostream>
#include <string>
#include <vector>
#include <future>
#include <XmlRpcValue.h>

static std::string title =
    "\n*********************************\n"
    "**  a.CreateDevice(Lidar only)  \n"
    "**  b.InitDevice                \n"
    "**  c.SetLaser                  \n"
    "**  d.SetEchoNumber             \n"
    "**  e.SetRawDataType            \n"
    "**  f.SetScanMode               \n"
    "**  g.SetPlayback               \n"
    "**  h.SetViewSpeed              \n"
    "**  i.SetDisconnect             \n"
    "**  j.Set Play state            \n"
    "** ****press q to quit********* **";

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    ROS_INFO("PointCloud Size:%d\n",msg->points.size());
}

void ClearParameter(ros::NodeHandle &m_node)
    {
       if(m_node.hasParam("update_device_parameter"))
       {
          std::string update_parameter;
          if(m_node.getParam("update_device_parameter",update_parameter))
          {
              m_node.deleteParam(update_parameter);
	   }
       	  m_node.deleteParam("update_device_parameter");
       }
    }

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"test_node");
    ros::NodeHandle node;
    ros::Subscriber cloud_sub=node.subscribe("cloud",100,PointCloudCallback);
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
                node.setParam("update_device_parameter","connect");
                std::cout << "Enter ip-address port: ";
                std::string ip;
                int port;
                std::cin >> ip >> port;
                XmlRpc::XmlRpcValue connect_xml;
                connect_xml["ip"]=ip;
                connect_xml["port"]=port;
                node.setParam("connect",connect_xml);
            }
                break;
            case 'b':
            {
                node.setParam("update_device_parameter","init_device");
            }
                break;
            case 'c':
            {
                node.setParam("update_device_parameter","laser_parameter");
                int level,factor,pulse_width;
                std::cout << "Enter level[0-19] factor[4-10] pulse_width[0-15]: ";
                std::cin >> level >> factor >> pulse_width;
                XmlRpc::XmlRpcValue Laser_xml;
                Laser_xml["level"]=level;
                Laser_xml["factor"]=factor;
                Laser_xml["pulse_width"]=pulse_width;
                node.setParam("laser_parameter",Laser_xml);
            }
                break;
            case 'd':
            {
                node.setParam("update_device_parameter","echo_number");
                std::cout << "Enter echo[1-4]: ";
                int echo;
                std::cin >> echo;
                node.setParam("echo_number",echo);
            }
                break;
            case 'e':
            {
                node.setParam("update_device_parameter","raw_data_type");
                std::cout << "Enter raw-data-type[0-1(0 means FPGA, 1 means DSP)]: ";
                int type;
                std::cin >> type;
                node.setParam("raw_data_type",type);
            }
                break;
            case 'f':
            {
                node.setParam("update_device_parameter","scan_mode");
                std::cout << "Enter scan-mode[0,1](0 means TWO-WAY, 1 means ONE-WAY): ";
                int mode;
                std::cin >> mode;
                node.setParam("scan_mode",mode);
            }
                break;
            case 'g':
            {
                node.setParam("update_device_parameter","playback");
                std::cout << "Enter playback file: ";
                std::string file;
                std::cin >> file;
                std::vector<std::string> file_list;
                file_list.push_back(file);
                node.setParam("playback",file_list);
            }
                break;
            case 'h':
            {
                node.setParam("update_device_parameter","view_parameter");
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
                node.setParam("view_parameter",view_xml);

            }
                break;
            case 'i':
            {
                node.setParam("update_device_parameter","disconnect_device");
            }
                break;
            case 'j':
            {
                node.setParam("update_device_parameter","play");
                std::cout << "Enter 1=play 2=pause 0=stop : ";
                int mode;
                std::cin >> mode;
                node.setParam("play",mode);
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


