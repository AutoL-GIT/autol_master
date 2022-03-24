#include "ros/ros.h"                            // ROS Default Header File

// #include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <signal.h>
#include "autol_driver/driver.h"


autol_driver::AutolDriver* dvr = NULL;

sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloudsrc)
{
  sensor_msgs::PointCloud2 cloudmsg;
  pcl::toROSMsg(cloudsrc, cloudmsg);
  cloudmsg.header.frame_id = "map";
  return cloudmsg;
}


void SigintHandler(int sig)
{
  dvr->Dispose();

  if(dvr != NULL)
    delete(dvr);

  ros::shutdown();
}

int main(int argc, char **argv)           // Node Main Function
{
  ros::init(argc, argv, "driver_node");   // Initializes Node Name
  ros::NodeHandle nh;                     // Node handle declaration for communication with ROS system
  ros::NodeHandle private_nh("~");        // Node handle declaration for communication with ROS system

  signal(SIGINT, SigintHandler);

  dvr = new autol_driver::AutolDriver(nh, private_nh);

  //if(dvr->ConnectUdpCom() == true)
    dvr->RecvPacket();








//  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("raw_pointcloud_data", 1);
//   while (ros::ok())
//   {
    
//     // if(dvr.LoopForData() == false)
//     // {
//     //   ROS_ERROR_THROTTLE(1.0, "Fail to get packet");
//     // }

//     //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //SAVE #1
//     pcl::PointCloud<pcl::PointXYZ> cloud; //SAVE #2


//     // // Fill in the cloud data
//     //cloud.width = 5;
//     // cloud.height = 1;
//     cloud.is_dense = false;
//     cloud.points.resize (5 * 1);
    



//     for (size_t i = 0; i < cloud.points.size(); ++i)
//     {
//       cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//       cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//       cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//     }
//     ROS_INFO("pos %f %f %f", cloud.points[0].x, cloud.points[0].y, cloud.points[0].z); 

//     sensor_msgs::PointCloud2 point_cloud2_msg = cloud2cloudmsg(cloud);

//   //ROS_INFO("recieve msg = %d", (int)cloud.points.size()); 
//     pub.publish(point_cloud2_msg);          
// //ROS_INFO_STREAM("aaaaaa"); 
//     ros::spinOnce();
//     //loop_rate.sleep();                      

//  }

  ROS_INFO(".... closing"); 

  return 0;
}

