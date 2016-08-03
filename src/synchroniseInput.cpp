#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<PointCloud2, Imu> SyncPolicy;
message_filters::Synchronizer<SyncPolicy>* cloud_sync;

ros::Publisher* cloud_pub_pointer;
ros::Publisher* imu_pub_pointer;


void callback(const sensor_msgs::PointCloud2::ConstPtr &cloudIn, const ImuConstPtr& imuIn)
{
  cloud_pub_pointer->publish(cloudIn);
  imu_pub_pointer->publish(imuIn);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_scan");

  ros::NodeHandle nh;

  int q = 20; // queue size

  std::string cloud_topic("/scan_cloud_translated");
  std::string imu_topic("/imu/data");

  message_filters::Subscriber<PointCloud2> cloud_sub(nh, cloud_topic, 2);
  message_filters::Subscriber<Imu> imu_sub(nh, imu_topic, 10);

  cloud_sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(q), cloud_sub, imu_sub);

  cloud_sync->registerCallback(boost::bind(&callback, _1, _2));

  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/sync_scan_cloud_filtered", 1);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu> ("/microstrain/imu", 1);

  cloud_pub_pointer = &cloud_pub;
  imu_pub_pointer = &imu_pub;
  
  ros::spin();

  return 0;
}