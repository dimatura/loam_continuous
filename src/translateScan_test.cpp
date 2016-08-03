#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

ros::Publisher* point_cloud_publisher_pointer;
tf::TransformListener* listener_pointer;

void transformScan(const sensor_msgs::LaserScan::ConstPtr &scan) {
  laser_geometry::LaserProjection lp;
  try {
      if(!listener_pointer->waitForTransform(
          scan->header.frame_id,
          "/base_link",
          scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
          ros::Duration(10.0))) {
          ROS_INFO_STREAM("SKIPPED! " << scan->header.seq);
          return;
      }
  } catch(tf::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          return;
  }

  
  sensor_msgs::PointCloud2 cloud;
  lp.transformLaserScanToPointCloud( "/base_link", *scan, cloud, *listener_pointer);
  cloud.header.stamp = scan->header.stamp;
  cloud.header.frame_id = "/base_link";
  point_cloud_publisher_pointer->publish(cloud);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "scan_translater");
  ros::NodeHandle nh;

  // default amount of time to cache the data
  tf::TransformListener listener(ros::Duration(10));

  ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan> ("/lidar_scan", 100, transformScan);

  // we'll transform a point once every 2.5e+7 nsecs
  // ros::Timer timer = nh.createTimer(ros::Duration(0.025), boost::bind(&transformScan, boost::ref(listener), boost::ref(scan_sub)));

  ros::Publisher point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered", 1);

  point_cloud_publisher_pointer = &point_cloud_publisher;

  ros::spin();

}