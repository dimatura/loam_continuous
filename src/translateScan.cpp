#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

ros::Publisher* point_cloud_publisher_pointer;
tf::TransformListener* listener_pointer;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr &scan) {
  laser_geometry::LaserProjection lp;
  try {
      if(!listener_pointer->waitForTransform(
          scan->header.frame_id,
          "/base_link",
          scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
          ros::Duration(1.0))) {
          return;
      }
  } catch(tf::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          return;
  }   
  sensor_msgs::PointCloud2 cloud;
  lp.transformLaserScanToPointCloud( "/base_link", *scan, cloud, *listener_pointer);
  point_cloud_publisher_pointer->publish(cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "translateScan");

  ros::NodeHandle nh;
  tf::TransformListener listener;

  message_filters::Subscriber<sensor_msgs::LaserScan> sub(nh, "/lidar_scan", 20);
  message_filters::TimeSequencer<sensor_msgs::LaserScan> seq(sub, ros::Duration(0.01), ros::Duration(0.001), 10);

  seq.registerCallback(boost::bind(&scanCallback, _1));

  ros::Publisher point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered", 2);

  point_cloud_publisher_pointer = &point_cloud_publisher;
  listener_pointer = &listener;

  ros::spin();
  return 0;
}