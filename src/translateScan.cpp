#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>

/**
 * Scan2PointTranslator
 * Translates the points acquired from LaserScan to PointCloud2
 */
class Scan2PointTranslator {
    public:
        Scan2PointTranslator();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        // void imuCallback(const sensor_msgs::Imu::ConstPtr& imuDataIn);
    private:
        ros::NodeHandle nh;
        tf::TransformListener listener;

        // laser
        laser_geometry::LaserProjection lp;
        ros::Publisher point_cloud_publisher;
        ros::Subscriber scan_sub;
        sensor_msgs::PointCloud2 filterCloud(const sensor_msgs::PointCloud2& scanPC);

        // imu
        // ros::Publisher imu_data_publisher;
        // ros::Subscriber imu_sub;
        // tf::TransformListener imu_listener;
        // tf::TransformBroadcaster tf_br;
};

Scan2PointTranslator::Scan2PointTranslator() {
    scan_sub = nh.subscribe<sensor_msgs::LaserScan> ("/lidar_scan", 2, &Scan2PointTranslator::scanCallback, this);
    point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/sync_scan_cloud_filtered", 2);

    // imu_sub = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 5, &Scan2PointTranslator::imuCallback, this);
    // imu_data_publisher = nh.advertise<sensor_msgs::Imu>("/microstrain/imu", 5);
}

void Scan2PointTranslator::scanCallback( const sensor_msgs::LaserScan::ConstPtr &scan ) {
  try {
      if(!listener.waitForTransform(
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
  lp.transformLaserScanToPointCloud( "/base_link", *scan, cloud, listener);
  point_cloud_publisher.publish(cloud);
}

// void Scan2PointTranslator::imuCallback(const sensor_msgs::Imu::ConstPtr &imuDataIn) {
//   tf::StampedTransform tf_imu_to_base;
//   try {
//     if(!imu_listener.waitForTransform(
//           imuDataIn->header.frame_id,
//           "/base_link",
//           imuDataIn->header.stamp,
//           ros::Duration(1.0))) {
//           return;
//       }
//       imu_listener.lookupTransform("/base_link", "/imu", ros::Time(0), tf_imu_to_base);
//   } catch(tf::TransformException &ex) {
//           ROS_ERROR("%s", ex.what());
//           ros::Duration(1.0).sleep();
//           return;
//   }

//   tf::Quaternion q;
//   q.setRPY(0, 0, 0);

//   tf_imu_to_base.setRotation(q);

//   tf_br.sendTransform(tf_imu_to_base);

  // tf_imu_to_base.setOrigin( tf::Vector3(0.19, 0.0, 0.149) );
  // tf::Quaternion q;
  // q.setRPY(0, 1.5705, 0);
  // tf_imu_to_base.setRotation(q);
  // tf_br.sendTransform(tf_imu_to_base);

  // tf::Matrix3x3 m(tf_imu_to_base.getRotation());
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  // ROS_INFO_STREAM("RPY:" << roll << ", " << pitch << ", " << yaw);



  // tf_imu_to_base.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
  // tf_imu_to_base.setRotation( tf::Quaternion(0, 0, 0, 1) );
  // tf_br.sendTransform(tf::StampedTransform(tf_imu_to_base, ros::Time::now(), "/imu/data", "/base_link"));

  // imu_data_publisher.publish(imu_msgs);
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "translateScan");

    Scan2PointTranslator s2pt;

    ros::spin();

    return 0;
}