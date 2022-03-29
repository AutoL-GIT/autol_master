#include <ros/ros.h>
#include <string>

#include <autol_msgs/AutolPacket.h>
#include <autol_msgs/AutolFrame.h>

#include "autol_driver/define.h"
#include "autol_driver/input_data.h"
#include "autol_driver/udp_socket.h"

namespace autol_driver
{
    #define BUF_SIZE pow(2, 20) * 1024

    class AutolDriver
    {
    public:
        AutolDriver(){};
        AutolDriver(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~AutolDriver(){};

        bool ConnectUdpCom();
        
        void RecvPacket();
        bool Dispose();
    private:
        struct
        {
            std::string manufacture_id;
            std::string model;
        }
        config_;
        
        boost::shared_ptr<InputData> input_data_;
        
        UDPSocket udp_socket;
        ros::Publisher pub_frame_;
        bool stop_udp_thread_ = false;
        int input_type_;
    };

}