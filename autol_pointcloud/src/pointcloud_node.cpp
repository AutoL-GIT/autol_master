#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>



#include "autol_pointcloud/convert.h"



  void cloud_cb (const autol_msgs::AutolFrame& frameMsg)
  {
      ROS_INFO("cloud_cb"); 
  }

  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    autol_pointcloud::Convert convert(nh, private_nh);

    //ros::Subscriber sub = nh.subscribe ("autol_frame_data", 10, cloud_cb);
    
    //ros::spin();

    return 0;
  }