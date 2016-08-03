#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


// conversion to pcl
#include <pcl/ros/conversions.h>
// pcl fromROSMsg() has changed, need to include <pcl_conversions/pcl_conversions.h> header
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


ros::Publisher* point_cloud_publisher_pointer;
tf::TransformListener* listener_pointer;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr &scan) {

  // transform laserscan to PointCloud2
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

  // transfer to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>()); // PointXYZ - Euclidean xyz coordinates
  pcl::fromROSMsg(cloud, *laserCloudIn); // convert from PointCloud2 to PointCloud

  // filter the pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDS(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
  downSizeFilter.setInputCloud(laserCloudIn);
  downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
  downSizeFilter.filter(*cloudDS);

  // convert from pcl and publish
  sensor_msgs::PointCloud2 cloudDS2;
  pcl::toROSMsg(*cloudDS, cloudDS2);
  cloudDS2.header.stamp = scan->header.stamp;
  cloudDS2.header.frame_id = "/base_link";
  point_cloud_publisher_pointer->publish(cloudDS2);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "translateScan");

  ros::NodeHandle nh;
  tf::TransformListener listener;

  message_filters::Subscriber<sensor_msgs::LaserScan> sub(nh, "/lidar_scan", 20);
  message_filters::TimeSequencer<sensor_msgs::LaserScan> seq(sub, ros::Duration(3), ros::Duration(0.01), 100);

  seq.registerCallback(boost::bind(&scanCallback, _1));

  ros::Publisher point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered", 2);

  point_cloud_publisher_pointer = &point_cloud_publisher;
  listener_pointer = &listener;

  ros::spin();
  return 0;
}