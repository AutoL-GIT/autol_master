#include "autol_pointcloud/rawdata.h"

namespace autol_data
{
    RawData::RawData() 
    {
        SetVerticalAngle(AUTOL_G32, CHANNEL_SIZE, VERTICAL_FOV);
    }

    void RawData::ConvertFromRaw(const autol_msgs::AutolPacket &packet, autol_data::ConvertDataBase& convert_data, bool is_slam_active)
    {
        //sleep(1);
        //usleep(10* 1000);

        UdpPacket* udp_packet = new UdpPacket();
        DeSerialize(udp_packet, (char*)&(&packet)->data[0]);
        float cur_top_bottom_offset = 0;

        if(udp_packet->header_.top_bottom_side_ == 0)
        {
            cur_top_bottom_offset = 0;
        }
        else
        {
            cur_top_bottom_offset = top_bottom_offset;
        }
        int cnt = 0; 

        for (size_t i = 0; i < DATABLOCK_SIZE; i++)
        {

            float azimuth = (float)udp_packet->data_block_[i].azimuth_ / 1000;

            for (size_t j = 0; j < CHANNEL_SIZE; j++)
            {
                unsigned int distance = (unsigned int)udp_packet->data_block_[i].channel_data_[j].tof_ * 4;
                uint8_t intensity = udp_packet->data_block_[i].channel_data_[j].intensity_;
                float vertical_angle = vertical_angle_arr_[j] + cur_top_bottom_offset;

                float pos_x, pos_y, pos_z;
                Get3DCoordinates(distance, vertical_angle, azimuth, 0, pos_x, pos_y, pos_z);
                
                pos_x /= 1000;
                pos_y /= 1000;
                pos_z /= 1000;

                // if(pos_x > 0.0 )
                // {
                //     ROS_INFO("ver %f %f", vertical_angle, pos_z); 
                //     ROS_INFO("pos %f %f %f", pos_x, pos_y, pos_z); 
                // }

                
                if(is_slam_active == true)
                {
                    int lidar_id = 0;
                    if(udp_packet->factory_ == 0x11)
                        lidar_id = 1;
                    else if(udp_packet->factory_ == 0x12)
                        lidar_id = 2;
                    
                    ApplyRPY(pos_x, pos_y, pos_z, lidar_id, calibration_.lidar_slamoffset_corrections);
                }
                convert_data.insertPoint(pos_x, pos_y, pos_z, 0, azimuth, distance, 0, 0);
                ++cnt;
            }
        }
        // if(udp_packet->header_.data_type_ == 0xA5B3C2AA)
        // {
        // }
        // else if(udp_packet->header_.data_type_ != 0xA5B3C2AA)
        // {
        // }

        // if(udp_packet->factory_ == 0x11)
        // {
        //     ROS_INFO("data %d ", udp_packet->factory_); 
        // }

        delete udp_packet;
        udp_packet = NULL;
    }

    void RawData::Get3DCoordinates(float  distance, float elevation, float azimuth_offset, float z_axes_offset, float& pos_x, float& pos_y, float& pos_z)
    {
        pos_x = distance *cos(elevation * PI / 180) * cos((azimuth_offset) * PI / 180);
        pos_y = distance *cos(elevation * PI / 180) * sin((azimuth_offset) * PI / 180);
        pos_z = distance *sin(elevation * PI / 180) + z_axes_offset;

    }

    void RawData::ApplyRPY(float& pos_x, float& pos_y, float& pos_z, int lidar_id, std::vector<autol_pointcloud::SlamOffset>& rpy)
	{
		float rotated_x = 0, rotated_y = 0, rotated_z = 0;

		switch (lidar_id)
		{
		case 1:
		{
			double tmp_x, tmp_y, tmp_z;

			double angle = rpy[0].roll * PI / 180.;
			rotated_y = pos_y * cos(angle) - pos_z * sin(angle);
			rotated_z = pos_y * sin(angle) + pos_z * cos(angle);

			pos_y = rotated_y;
			pos_z = rotated_z;

			angle = rpy[0].pitch * PI / 180.;
			rotated_x = pos_z * sin(angle) + pos_x * cos(angle);
			rotated_z = pos_z * cos(angle) - pos_x * sin(angle);

			pos_x = rotated_x;
			pos_z = rotated_z;
			angle = rpy[0].yaw * PI / 180.;
			rotated_x = pos_x * cos(angle) - pos_y * sin(angle);
			rotated_y = pos_x * sin(angle) + pos_y * cos(angle);

			pos_x = rotated_x;
			pos_y = rotated_y;

			rotated_y = rotated_y + rpy[0].y_offset;
			rotated_x = rotated_x + rpy[0].x_offset;
			rotated_z = rotated_z + rpy[0].z_offset;


			pos_x = rotated_x;
			pos_y = rotated_y;
			pos_z = rotated_z;
		}
		break;

		case 2:
		{
			double tmp_x, tmp_y, tmp_z;
			double angle = rpy[1].roll * PI / 180.;
			rotated_y = pos_y * cos(angle) - pos_z * sin(angle);
			rotated_z = pos_y * sin(angle) + pos_z * cos(angle);

			pos_y = rotated_y;
			pos_z = rotated_z;

			angle = rpy[1].pitch * PI / 180.;
			rotated_x = pos_z * sin(angle) + pos_x * cos(angle);
			rotated_z = pos_z * cos(angle) - pos_x * sin(angle);

			pos_x = rotated_x;
			pos_z = rotated_z;
			angle = rpy[1].yaw * PI / 180.;
			rotated_x = pos_x * cos(angle) - pos_y * sin(angle);
			rotated_y = pos_x * sin(angle) + pos_y * cos(angle);

			pos_x = rotated_x;
			pos_y = rotated_y;

			rotated_y = rotated_y + rpy[1].y_offset;
			rotated_x = rotated_x + rpy[1].x_offset;
			rotated_z = rotated_z + rpy[1].z_offset;

			pos_x = rotated_x;
			pos_y = rotated_y;
			pos_z = rotated_z;
		}
		break;

		default:
			break;
		}
	}

    void RawData::SetVerticalAngle(DeviceID device_id, int num_of_channel, float angle)
    {
        switch (device_id)
        {
            case DeviceID::AUTOL_G32:
            {

                top_bottom_offset = angle / (2 * num_of_channel);

                float angle_start = -angle / 2 + top_bottom_offset / 2;

                for (size_t i = 0; i < num_of_channel; i++)
                {
                    vertical_angle_arr_[i] = angle_start + (angle / num_of_channel) * i;
                    vertical_angle_arr_[i + 16] = angle_start + (angle / num_of_channel) * i + top_bottom_offset;
                }
            }
            break;
            default:
                break;
        }
    }

    void RawData::DeSerialize(UdpPacket* udp_packet, char* bytes)
	{
		memcpy(udp_packet, bytes, 1330);
	}

    int RawData::pointsPerPacket()
    {
        return DATABLOCK_SIZE * CHANNEL_SIZE;
    }

    void RawData::Setup()
    {
        //calibration_.Read("/home/autol/catkin_ws/src/autol_master/autol_pointcloud/params/64e_utexas.yaml");
        calibration_.ReadSlamOffset("/home/autol/catkin_ws/src/autol_master/autol_pointcloud/params/slam_offset.yaml");


        //rpy_.push_back(autol_pointcloud::SlamOffset({1, 1, 1,1, 1, 1}));
    }
}