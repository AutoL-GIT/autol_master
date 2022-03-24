#include "autol_driver/driver.h"



#include <tf/transform_listener.h>


namespace autol_driver
{
    
    AutolDriver::AutolDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        ROS_INFO("constructor : AutolDriver"); 

        string pcap_path;
        int input_type;
        private_nh.param("manufacture_id", config_.manufacture_id, std::string("autol"));
        private_nh.param("model_id", config_.model, std::string("G32"));
        private_nh.param("pcap_path", pcap_path, std::string(""));
        private_nh.param("input_type", input_type, 2);

        pub_frame_ = nh.advertise<autol_msgs::AutolFrame>("autol_frame_data", 10);
      
        if (input_type == 1)   
        {            
            input_data_.reset(new autol_driver::SocketInput(private_nh, UDP_PORT));
        }
        else if (input_type == 2)
        {               
            input_data_.reset(new autol_driver::PcapInput(private_nh, UDP_PORT, FRAMERATE * PACKET_PER_SECOND, pcap_path));                  
        }    
    }

    bool AutolDriver::Dispose()
    {
        return true;
    }
    
    void AutolDriver::RecvPacket()
    {
        ROS_INFO("RecvPacket called"); 

        UdpPacket* packet = new UdpPacket();

        autol_msgs::AutolPacket tmp_packet;
        autol_msgs::AutolFramePtr lidar_frame(new autol_msgs::AutolFrame);
        autol_msgs::AutolFramePtr lidar1_frame(new autol_msgs::AutolFrame);
        autol_msgs::AutolFramePtr lidar2_frame(new autol_msgs::AutolFrame);

        bool is_lidar1_active = false;
        bool is_lidar2_active = false;
        unsigned int lidar1_frame_count = 0;
        unsigned int lidar2_frame_count = 0;

        bool is_first_fov_lidar1 = true;
        bool is_first_fov_lidar2 = true;
        int lidar1_stage_count = 0;
        int lidar2_stage_count = 0;

        lidar_frame->packets.reserve(PACKET_PER_SECOND);
        lidar1_frame->packets.reserve(PACKET_PER_SECOND);
        lidar2_frame->packets.reserve(PACKET_PER_SECOND);
        

        while(ros::ok())
        {
            if(input_data_->is_ready_input_ == false)
            {                
                break;
            }

            int ret = input_data_->getPacket(&tmp_packet, PACKET_DATA_SIZE);

            if (ret != PACKET_DATA_SIZE)
            {
                continue;
            }
        
            memcpy(packet, (char*)&(&tmp_packet)->data[0], PACKET_DATA_SIZE);

            if (packet->factory_ == 0x11)
            {
                is_lidar1_active = true;
                lidar1_frame->packets.push_back(tmp_packet);
            }
            else if (packet->factory_ == 0x12)
            {
                is_lidar2_active = true;
                lidar2_frame->packets.push_back(tmp_packet);
            }

            if (packet->header_.data_type_ == 0xA5B3C2AA && packet->header_.packet_id_ != 0)
            {
                if (packet->factory_ == 0x11)
                {
                    ++lidar1_stage_count;

                    if(is_first_fov_lidar1 == true)
                    {
                        lidar1_frame.reset(new autol_msgs::AutolFrame);
                        lidar1_stage_count = 0;
                        is_first_fov_lidar1 = false;               
                    }
                    if(lidar1_stage_count >= 2)
                    {
                        if(lidar1_frame_count == 0)
                        {
                            lidar_frame->packets.insert(lidar_frame->packets.end(), lidar1_frame->packets.begin(), lidar1_frame->packets.end());
                        }
                        lidar1_frame.reset(new autol_msgs::AutolFrame);
                        lidar1_frame->packets.reserve(PACKET_PER_SECOND);

                        lidar1_stage_count = 0;
                        ++lidar1_frame_count;
                    }
                }
                else if (packet->factory_ == 0x12)
                {
                    ++lidar2_stage_count;

                    if(is_first_fov_lidar2 == true)
                    {
                        lidar2_frame.reset(new autol_msgs::AutolFrame);
                        lidar2_stage_count = 0;
                        is_first_fov_lidar2 = false;               
                    }

                    if(lidar2_stage_count >= 2)
                    {
                        if(lidar2_frame_count == 0)
                        {
                            lidar_frame->packets.insert(lidar_frame->packets.end(), lidar2_frame->packets.begin(), lidar2_frame->packets.end());
                        }
                        lidar2_frame.reset(new autol_msgs::AutolFrame);
                        lidar2_frame->packets.reserve(PACKET_PER_SECOND);

                        lidar2_stage_count = 0;
                        ++lidar2_frame_count;
                    }
                }
                
                if(lidar1_frame_count > 50 || lidar2_frame_count > 50)
                {
                    lidar1_frame_count = 0;
                    lidar2_frame_count = 0;

                    int active_lidar_count = 2;
                    if(lidar1_frame_count == 0)
                    {
                        is_lidar1_active = false;
                        --active_lidar_count;
                    }
                    if(lidar2_frame_count == 0)
                    {
                        is_lidar2_active = false;
                        --active_lidar_count;
                    }
                    lidar_frame.reset(new autol_msgs::AutolFrame);
                    lidar_frame->packets.reserve(active_lidar_count * PACKET_PER_SECOND);
                }

                if(((is_lidar1_active == true && lidar1_frame_count > 0) || is_lidar1_active == false) &&
                   ((is_lidar2_active == true && lidar2_frame_count > 0) || is_lidar2_active == false) &&
                   (is_lidar1_active == true || is_lidar2_active == true))
                {
                    int active_lidar_count = 0;

                    if(is_lidar1_active == true && is_lidar2_active == true)
                    {
                        input_data_->changePacketRate(FRAMERATE * PACKET_PER_SECOND * 2);
                        active_lidar_count = 2;
                    }
                    else
                    {
                        input_data_->changePacketRate(FRAMERATE * PACKET_PER_SECOND * 1);
                        if(is_lidar1_active == true)
                            ++active_lidar_count;
                        if(is_lidar2_active == true)
                            ++active_lidar_count;
                    }

                    if(lidar_frame->packets.size() == active_lidar_count * PACKET_PER_SECOND)
                    {
                        pub_frame_.publish(lidar_frame);
                    }
                    lidar_frame.reset(new autol_msgs::AutolFrame);
                    lidar_frame->packets.reserve(active_lidar_count * PACKET_PER_SECOND);

                    lidar1_frame_count = 0;
                    lidar2_frame_count = 0;
                }
            }
        }

        delete packet;
        packet = NULL;
    }

}