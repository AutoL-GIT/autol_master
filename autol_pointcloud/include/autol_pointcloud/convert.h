
#include <ros/ros.h>
#include <autol_msgs/AutolPacket.h>

#include "autol_pointcloud/rawdata.h"
#include "autol_pointcloud/convertdatabase.h"
#include "autol_pointcloud/pointcloudXYZ.h"

#include "autol_pointcloud/calibration.h"



namespace autol_pointcloud
{
    class Convert
    {
    public:
        Convert(ros::NodeHandle nh, ros::NodeHandle private_nh);
        ~Convert(){}

    private:
        void frameData_cb (const autol_msgs::AutolFrame::ConstPtr &frameMsg);

        boost::shared_ptr<autol_data::RawData> raw_data_;
        boost::shared_ptr<autol_data::ConvertDataBase> convert_data_;

        ros::Publisher pub_autol_pointcloud_;

        bool is_slam_active_ = false;
        int frame_cnt = 0;
    };
}