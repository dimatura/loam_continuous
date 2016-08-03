#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <sstream>
#include <ros/package.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "publish_scene");

  ros::NodeHandle n;

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 100);
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered", 40);
  ros::Rate loop_rate(100);

  std::string scene_path = "/home/edbot/logs/husky/2016-07-13-outdoor-george-square-west/run1/fixed_timestamps/";
  std::string filename = "high_fidelity_husky_outdoor.bag";

  while (ros::ok()){
    ROS_INFO_STREAM("Start playing " << filename);

    rosbag::Bag bag;
    bag.open(scene_path + filename, rosbag::bagmode::Read);

    std::string imu_topic = "/imu/data";
    std::string cloud_topic = "/sync_scan_cloud_filtered";

    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(cloud_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // iterate the bag messages
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == imu_topic) 
      {
        sensor_msgs::Imu::ConstPtr imu_ptr = m.instantiate<sensor_msgs::Imu>();
        if (imu_ptr != NULL) {
          imu_pub.publish(*imu_ptr);
        }
      }

      if (m.getTopic() == cloud_topic) 
      {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
          cloud_pub.publish(*cloud_ptr);
      }

      loop_rate.sleep();

    }

  bag.close();
  ros::spinOnce();
  ROS_INFO_STREAM("Done.");
  return 0;
  }
}