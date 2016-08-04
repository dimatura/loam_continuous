#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <sstream>
#include <ros/package.h>
#include <tf2_msgs/TFMessage.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "publish_scene");

  ros::NodeHandle nh;

  std::string rosbag;
  
  nh.getParam("/rosbag", rosbag);

  ROS_INFO_STREAM("Start playing " << rosbag);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered", 40);
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/lidar_scan", 40);
  ros::Publisher tf_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 285);
  ros::Rate loop_rate(50);

  while (ros::ok()){
    rosbag::Bag bag;
    bag.open(rosbag, rosbag::bagmode::Read);

    std::string imu_topic = "/imu/data";
    std::string laser_topic = "/lidar_scan";
    std::string cloud_topic = "/sync_scan_cloud_filtered";
    std::string tf_topic = "/tf";

    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(cloud_topic);
    topics.push_back(laser_topic);
    topics.push_back(tf_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // iterate the bag messages
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == imu_topic) 
      {
        sensor_msgs::Imu::ConstPtr imu_ptr = m.instantiate<sensor_msgs::Imu>();
        if (imu_ptr != NULL)
          imu_pub.publish(*imu_ptr);
      }

      if (m.getTopic() == cloud_topic) 
      {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
          cloud_pub.publish(*cloud_ptr);
      }

      if (m.getTopic() == laser_topic) 
      {
        sensor_msgs::LaserScan::ConstPtr laser_ptr = m.instantiate<sensor_msgs::LaserScan>();
        if (laser_ptr != NULL)
          laser_pub.publish(*laser_ptr);
      }

      if (m.getTopic() == tf_topic) 
      {
        tf2_msgs::TFMessage::ConstPtr tf_ptr = m.instantiate<tf2_msgs::TFMessage>();
        if (tf_ptr != NULL)
          tf_pub.publish(*tf_ptr);
      }

      loop_rate.sleep();

    }

  bag.close();
  ros::spinOnce();
  ROS_INFO_STREAM("Done.");
  return 0;
  }
}