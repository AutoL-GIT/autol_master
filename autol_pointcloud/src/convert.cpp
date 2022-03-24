#include "autol_pointcloud/convert.h"



namespace autol_pointcloud
{
    Convert::Convert(ros::NodeHandle nh, ros::NodeHandle private_nh):
    raw_data_(new autol_data::RawData())
    {
        ROS_INFO("Convert()"); 

        private_nh.param("slam", is_slam_active_, false);

        raw_data_->Setup();

        convert_data_ = boost::shared_ptr<autol_data::PointcloudXYZ>(new autol_data::PointcloudXYZ(0, 0, "map", "map", raw_data_->pointsPerPacket()));

        pub_autol_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2> ("autol_pointcloud", 1);
        ros::Subscriber sub_frame = nh.subscribe ("autol_frame_data", 10, &Convert::frameData_cb, (Convert *)this,
            ros::TransportHints().tcpNoDelay(true));

        ros::spin ();
    }

    void Convert::frameData_cb (const autol_msgs::AutolFrame::ConstPtr &frameMsg)
    {
        convert_data_->Setup(frameMsg);


        for (size_t i = 0; i < frameMsg->packets.size(); ++i)
        {
            raw_data_->ConvertFromRaw(frameMsg->packets[i], *convert_data_, is_slam_active_);
        }
        //ROS_INFO("min_azimuth %f", min_azimuth);
       // ROS_INFO("packets %d", (int)frameMsg->packets.size());


        pub_autol_pointcloud_.publish(convert_data_->resizeData());



        //ROS_INFO("%d %d", CLOCKS_PER_SEC, TIME_UTC);

    }
}