#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

/**
 * Scan2PointTranslator
 * Translates the points acquired from LaserScan to PointCloud2
 */
class Scan2PointTranslator {
    public:
        Scan2PointTranslator();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    private:
        ros::NodeHandle nh;
        laser_geometry::LaserProjection lp;
        tf::TransformListener listener;

        ros::Publisher point_cloud_publisher;
        ros::Subscriber scan_sub;
};

Scan2PointTranslator::Scan2PointTranslator() {
    scan_sub = nh.subscribe<sensor_msgs::LaserScan> ("/lidar_scan", 2, &Scan2PointTranslator::scanCallback, this);
    point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered", 2);
}

void Scan2PointTranslator::scanCallback( const sensor_msgs::LaserScan::ConstPtr &scan ) {
  try {
      if(!listener.waitForTransform(
          scan->header.frame_id,
          "/base_link",
          scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
          ros::Duration(1.0))){
        return;
      }
  } catch(tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      return;
  }
     
    sensor_msgs::PointCloud2 cloud;
    lp.transformLaserScanToPointCloud( "/base_link", *scan, cloud, listener);
    point_cloud_publisher.publish(cloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "translateScan");

  Scan2PointTranslator s2pt;

  ros::spin();

  return 0;
}