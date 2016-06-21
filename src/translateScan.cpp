#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "filters/filter_chain.h"

/**
 * Scan2PointTranslator
 * Translates the points acquired from LaserScan to PointCloud2
 */
class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  sensor_msgs::LaserScan filtered_scan;

  // Filter Chain
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "lidar_scan", 10),
    laser_notifier_(laser_sub_,listener_, "head", 10),
    filter_chain_("sensor_msgs::LaserScan")
  {
    // Configure filter chain
    filter_chain_.configure("");

    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

    // Run the filter chain
    filter_chain_.update (*scan_in, filtered_scan);
    try {
        if(!listener_.waitForTransform(
            scan_in->header.frame_id,
            "/head",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
            ros::Duration(1.0))) {
            return;
        }
    } catch(tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
    }
     
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud( "/head", *scan_in, cloud, listener_);
    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "translateScan");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}