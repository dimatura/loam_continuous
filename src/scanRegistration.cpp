#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/ros/conversions.h>
// pcl fromROSMsg() has changed, need to include <pcl_conversions/pcl_conversions.h> header
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/PoseStamped.h>

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

double initTime;
double timeStart;
double timeLasted;
bool systemInited = false;

double timeScanCur = 0;
double timeScanLast = 0;

int laserRotDir = 1;
float mean_sweep = 0; 

int skipFrameNum = 5;
int skipFrameCount = 0;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLessExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());

sensor_msgs::PointCloud2 laserCloudExtreCur2;
sensor_msgs::PointCloud2 laserCloudLast2;

ros::Publisher* pubLaserCloudExtreCurPointer;
ros::Publisher* pubLaserCloudLastPointer;

int cloudSortInd[1200];
int cloudNeighborPicked[1200];

int imuPointerFront = 0;
int imuPointerLast = -1;
// GT changes 1/07/16 - IMU HZ
// const int imuQueLength = 50;
const int imuQueLength = 100;
bool imuInited = false;

float imuRollStart, imuPitchStart, imuYawStart;
float imuRollCur, imuPitchCur, imuYawCur;

float imuVeloXStart, imuVeloYStart, imuVeloZStart;
float imuShiftXStart, imuShiftYStart, imuShiftZStart;
float imuVeloXCur, imuVeloYCur, imuVeloZCur;
float imuShiftXCur, imuShiftYCur, imuShiftZCur;

float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

double imuAccuRoll = 0;
double imuAccuPitch = 0;
double imuAccuYaw = 0;

// debug purpose publishers
// ros::Publisher* pubCornerPointsSharpPointer;
// ros::Publisher* pubCornerPointsLessSharpPointer;
// ros::Publisher* pubSurfPointsFlatPointer;
// ros::Publisher* pubSurfPointsLessFlatPointer;
// ros::Publisher* pubImuTransPointer;
// ros::Publisher* pubLaserAnglePointer;
// std_msgs::Float32 laserAngle2;
// ros::Publisher* pubFirstPointPointer;
// ros::Publisher* pubLastPointPointer;
// ros::Publisher* pubImuVisPointer;
// ros::Publisher* pubImuVisPointer2;
// end debug

// transfer laserscan to pointcloud2
tf::TransformListener *listener_pointer;
// laser
laser_geometry::LaserProjection lp;

void ShiftToStartIMU()
{
  float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

  float x2 = x1;
  float y2 =  cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuShiftFromStartXCur =  cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}

void VeloToStartIMU()
{
  float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuVeloFromStartZCur = z2;
}

void TransformToStartIMU(pcl::PointXYZHSV *p)
{
  // rotation over z
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

  // rotation over x
  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

  // rotation over y
  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;


  float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
  float y4 = y3;
  float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

  float x5 = x4;
  float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
  float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

  p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
  p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}

/*
 * Calculates the drift
 * The velocity for the last measurement in X, Y and Z directions
 * stored in 3 arrays:
 * imuVeloX
 * imuVeloY
 * imuVeloZ
 */
void AccumulateIMUShift()
{
  // retrieve the RPY and the acceleration
  // comments are as follows: 
  // the working example with multisense // current values
  float roll = imuRoll[imuPointerLast]; // yaw // cur:roll
  float pitch = imuPitch[imuPointerLast]; // pitch // cur:-pitch
  float yaw = imuYaw[imuPointerLast]; // roll // cur: -imuAccuYaw
  float accX = imuAccX[imuPointerLast]; // linear_acceleration.y // cur: -linear_acceleration.y
  float accY = imuAccY[imuPointerLast]; // linear_acceleration.x+9.81 // cur: linear_acceleration.x+9.81
  float accZ = imuAccZ[imuPointerLast]; // linear_acceleration.z // cur: linear_acceleration.z

  // ROS_INFO_STREAM("accX, accY, accZ: (" << accX << ", " << accY << ", " << accZ << ")");
  // ROS_INFO_STREAM("RPY: (" << roll << ", " << pitch << ", " << yaw << ")");

  // roll is around z axis ??
  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;

  // pitch is around x axis?
  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  // yaw is around y axis?
  accX =  cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < 0.1) {

    // equations of motion to calculate displacement in x y and z (deltaS = ut+1/2(at^2))
    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;

    // calculate the new velocity (v = u+at)
    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

    // ROS_INFO_STREAM("IMU SHIFT (X,Y,Z): (" << imuShiftX[imuPointerLast] << ", " << imuShiftY[imuPointerLast] << ", " << imuShiftZ[imuPointerLast] << ")");
    // ROS_INFO_STREAM("IMU VELO (X,Y,Z): (" << imuVeloX[imuPointerLast] << ", " << imuVeloY[imuPointerLast] << ", " << imuVeloZ[imuPointerLast] << ")");
  }
}

/**
 * The message from the point cloud go here.
 */
void laserCloudHandler(sensor_msgs::PointCloud2 laserCloudIn2)
{
  // ROS_INFO_STREAM("Handler initiated: " << ros::Time::now());
  if (!systemInited) {
    initTime = laserCloudIn2.header.stamp.toSec(); // initialize the start of the point cloud
    imuPointerFront = (imuPointerLast + 1) % imuQueLength;
    systemInited = true;
  }

  timeScanLast = timeScanCur; // initially 0
  timeScanCur = laserCloudIn2.header.stamp.toSec(); // timestamp of measurement
  timeLasted = timeScanCur - initTime; // first measurement is 0, then record the time from the very first measurements (system init)
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>()); // PointXYZ - Euclidean xyz coordinates
  pcl::fromROSMsg(laserCloudIn2, *laserCloudIn); // convert from PointCloud2 to PointCloud
  int cloudInSize = laserCloudIn->points.size();
  // ROS_INFO_STREAM("cloudInSize: " << cloudInSize);

  int cloudSize = 0;
  pcl::PointXYZHSV laserPointIn;
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>()); // PointXYZHSV - xyz with hue, saturation and value

  // fill the new pointcloud laserCloud (XYZHSV) with the laserCloudIn (PointCloud2) via the var laserPointIn (PointXYZHSV)
  for (int i = 0; i < cloudInSize; i++) {
    // adopt the x y z values
    laserPointIn.x = laserCloudIn->points[i].x;
    laserPointIn.y = laserCloudIn->points[i].y;
    laserPointIn.z = laserCloudIn->points[i].z;
    laserPointIn.h = timeLasted; // hue - time from the initialization of the system, saturation is calculated afterwards
    laserPointIn.v = 0; // value is always 0, as this is the definition for obtained point

    if (!(fabs(laserPointIn.x) < 1.5 && fabs(laserPointIn.y) < 1.5 && fabs(laserPointIn.z) < 1.5)) {
      laserCloud->push_back(laserPointIn); // push a new point at the end of the container (?) 
      cloudSortInd[cloudSize] = cloudSize;
      cloudNeighborPicked[cloudSize] = 0;
      cloudSize++;
    }
  }

  // ROS_INFO("The size of the cloud points incomming is: %d, new cloudSize is: %d", cloudInSize, cloudSize);

  // get the first and last laser points
  pcl::PointXYZ laserPointFirst = laserCloudIn->points[0];
  pcl::PointXYZ laserPointLast = laserCloudIn->points[cloudInSize - 1];

  // spherical coordinates
  // first point range = sqrt(x^2 + y^2 + z^2) 
  float rangeFirst = sqrt(laserPointFirst.x * laserPointFirst.x + laserPointFirst.y * laserPointFirst.y
                 + laserPointFirst.z * laserPointFirst.z);
  // normalize (?) x y z
  laserPointFirst.x /= rangeFirst;
  laserPointFirst.y /= rangeFirst;
  laserPointFirst.z /= rangeFirst;

  // last point range = sqrt(x^2 + y^2 + z^2)
  float rangeLast = sqrt(laserPointLast.x * laserPointLast.x + laserPointLast.y * laserPointLast.y
                 + laserPointLast.z * laserPointLast.z);
  laserPointLast.x /= rangeLast;
  laserPointLast.y /= rangeLast;
  laserPointLast.z /= rangeLast;

  // DEBUG prupose
  // show in topic the first and last points and then the angle that is being calculated
  // ROS_INFO_STREAM("Cloud size: " << cloudSize);
  // ROS_INFO_STREAM("last: " << laserPointLast);
  // ROS_INFO_STREAM("first: " << laserPointFirst);
  
  // push the first point to its own pointcloud
  // pcl::PointCloud<pcl::PointXYZ>::Ptr laserFirstPointCloud(new pcl::PointCloud<pcl::PointXYZ>());
  // laserFirstPointCloud->push_back(laserPointFirst);

  // sensor_msgs::PointCloud2 laserPointFirstCloud2;
  // pcl::toROSMsg(*laserFirstPointCloud, laserPointFirstCloud2);
  // laserPointFirstCloud2.header.stamp = laserCloudIn2.header.stamp;
  // laserPointFirstCloud2.header.frame_id = "/camera";
  // pubFirstPointPointer->publish(laserPointFirstCloud2);

  // // and the last point to its own pointcloud
  // pcl::PointCloud<pcl::PointXYZ>::Ptr laserLastPointCloud(new pcl::PointCloud<pcl::PointXYZ>());
  // laserLastPointCloud->push_back(laserPointLast);

  // sensor_msgs::PointCloud2 laserPointLastCloud2;
  // pcl::toROSMsg(*laserLastPointCloud, laserPointLastCloud2);
  // laserPointLastCloud2.header.stamp = laserCloudIn2.header.stamp;
  // laserPointLastCloud2.header.frame_id = "/camera";
  // pubLastPointPointer->publish(laserPointLastCloud2);
  // end debug

  // the angle of the laser
  // atan2(x,y) - angle b/w two coordinates x, y 
  // supposedly this is the angle the laser is showing
  float laserAngle = atan2(laserPointLast.x - laserPointFirst.x, laserPointLast.y - laserPointFirst.y);

  // DEBUG purpose
  // publish the laser angle as a channel to be plotted
  // laserAngle2.data = laserAngle*rad2deg;
  // pubLaserAnglePointer->publish(laserAngle2);
  // end debug

  // ROS_INFO("Laser angle: %f", (laserAngle * rad2deg));
  bool newSweep = false;
  if (laserAngle * laserRotDir < 0 && timeLasted - timeStart > 2.0) {
    laserRotDir *= -1;
    newSweep = true;
    // ROS_INFO("New sweep!! Laser angle: %f", (laserAngle * rad2deg));
  }

  // reinitalise the values if there's a new sweep
  if (newSweep) {
    timeStart = timeScanLast - initTime; // compute the starting time of this particular scan

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = imuPitchStart;
    imuTrans->points[0].y = imuYawStart;
    imuTrans->points[0].z = imuRollStart;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = imuPitchCur;
    imuTrans->points[1].y = imuYawCur;
    imuTrans->points[1].z = imuRollCur;
    imuTrans->points[1].v = 11;

    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    *laserCloudExtreCur += *laserCloudLessExtreCur; // add the laserCloudLessExtreCur to laserCloudExtreCur
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudLast2); // translate laserCloudExtreCur + imuTrans (pcl:PointCloud) to sensorMsg::PointCloud2
    laserCloudLast2.header.stamp = ros::Time().fromSec(timeScanLast);
    laserCloudLast2.header.frame_id = "/camera";
    laserCloudExtreCur->clear();
    laserCloudLessExtreCur->clear();
    imuTrans->clear();

    imuRollStart = imuRollCur;
    imuPitchStart = imuPitchCur;
    imuYawStart = imuYawCur;

    imuVeloXStart = imuVeloXCur;
    imuVeloYStart = imuVeloYCur;
    imuVeloZStart = imuVeloZCur;

    imuShiftXStart = imuShiftXCur;
    imuShiftYStart = imuShiftYCur;
    imuShiftZStart = imuShiftZCur;
  }

  // reset the imu values
  imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;
  imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
  imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;
  if (imuPointerLast >= 0) {
    while (imuPointerFront != imuPointerLast) {
      if (timeScanCur < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (timeScanCur > imuTime[imuPointerFront]) {
      imuRollCur = imuRoll[imuPointerFront];
      imuPitchCur = imuPitch[imuPointerFront];
      imuYawCur = imuYaw[imuPointerFront];

      imuVeloXCur = imuVeloX[imuPointerFront];
      imuVeloYCur = imuVeloY[imuPointerFront];
      imuVeloZCur = imuVeloZ[imuPointerFront];

      imuShiftXCur = imuShiftX[imuPointerFront];
      imuShiftYCur = imuShiftY[imuPointerFront];
      imuShiftZCur = imuShiftZ[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
      float ratioFront = (timeScanCur - imuTime[imuPointerBack]) 
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - timeScanCur) 
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
      } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
      } else {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
      }

      imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
      imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
      imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

      imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
      imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
      imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
    }
  }

  // initiate the very first imu measurement
  if (!imuInited) {
    imuRollStart = imuRollCur;
    imuPitchStart = imuPitchCur;
    imuYawStart = imuYawCur;

    imuVeloXStart = imuVeloXCur;
    imuVeloYStart = imuVeloYCur;
    imuVeloZStart = imuVeloZCur;

    imuShiftXStart = imuShiftXCur;
    imuShiftYStart = imuShiftYCur;
    imuShiftZStart = imuShiftZCur;

    imuInited = true;
  }

  // calculate the shift from the starting imu
  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * (timeLasted - timeStart);
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * (timeLasted - timeStart);
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * (timeLasted - timeStart);

  //float befImuShiftFromStartXCur = imuShiftFromStartXCur;
  //float befImuShiftFromStartYCur = imuShiftFromStartYCur;
  //float befImuShiftFromStartZCur = imuShiftFromStartZCur;
  ShiftToStartIMU();
  //ROS_INFO("Imu Shift X: (bef, %f), (aft, %f); Y: (bef, %f), (aft, %f), Z: (bef, %f), (aft, %f)", befImuShiftFromStartXCur, imuShiftFromStartXCur, befImuShiftFromStartYCur, imuShiftFromStartYCur, befImuShiftFromStartZCur, imuShiftFromStartZCur);


  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  VeloToStartIMU();

  for (int i = 0; i < cloudSize; i++) {
    TransformToStartIMU(&laserCloud->points[i]);
  }

  // calculate smoothness value based on the 5 neighbours on each side 
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;
    
    // calculate smoothness
    laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ;
  }
  
  // float m = 0;
  for (int i = 5; i < cloudSize - 6; i++) {
    // calculate difference with the neighbour
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    // incremental average calculation of the diff
    // m = m + ((diff-m)/(cloudSize-12));
    // if there's a big difference
    if (diff > 0.05) {

      // depth1 - current point's score
      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);
      // depth2 - neighbour point's score
      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      // filter out points that are picked based on depth
      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }

      // ROS_INFO("Point %d:", i);
      // for (int pp=0; pp < cloudSize; pp++) {
      //   ROS_INFO("N: %d", cloudNeighborPicked[pp]);
      // }
      // ros::Duration(5.0).sleep();
    }

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }
  // ROS_INFO("avrg difference: %f", m);
  // if(newSweep) {
  //   mean_sweep = mean_sweep + ((m-mean_sweep)/10);
  //   ROS_INFO("avrg per sweep: %f", mean_sweep);
  // }

  // define the edge and surface areas
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());

  int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0), 
                        6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
  int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0), 
                      5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

  for (int i = 0; i < 4; i++) {
    int sp = startPoints[i];
    int ep = endPoints[i];

    // sort based on the smoothness value
    for (int j = sp + 1; j <= ep; j++) {
      for (int k = j; k >= sp + 1; k--) {
        if (laserCloud->points[cloudSortInd[k]].s < laserCloud->points[cloudSortInd[k - 1]].s) {
          int temp = cloudSortInd[k - 1];
          cloudSortInd[k - 1] = cloudSortInd[k];
          cloudSortInd[k] = temp;
        }
      }
    }

    // select the maximum smoothness - edge points
    int largestPickedNum = 0;
    for (int j = ep; j >= sp; j--) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s > 0.1 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {
        
        largestPickedNum++;
        if (largestPickedNum <= 2) {
          // value of 2 means it's a an edge
          laserCloud->points[cloudSortInd[j]].v = 2;
          cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else if (largestPickedNum <= 20) {
          laserCloud->points[cloudSortInd[j]].v = 1;
          cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else {
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 5; k++) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k - 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k - 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
        for (int k = -1; k >= -5; k--) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k + 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k + 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
      }
    }

    // select the minimum smoothness - planar points
    int smallestPickedNum = 0;
    for (int j = sp; j <= ep; j++) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s < 0.1 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {

        laserCloud->points[cloudSortInd[j]].v = -1;
        surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);

        smallestPickedNum++;
        if (smallestPickedNum >= 4) {
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 5; k++) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k - 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k - 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
        for (int k = -1; k >= -5; k--) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k + 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k + 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
      }
    }
  }

  for (int i = 0; i < cloudSize; i++) {
    if (laserCloud->points[i].v == 0) {
      surfPointsLessFlat->push_back(laserCloud->points[i]);
    }
  }

  // downsize filter of the planar areas (voxel in pcl)
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
  downSizeFilter.setInputCloud(surfPointsLessFlat);
  downSizeFilter.setLeafSize(0.3, 0.3, 0.3);
  downSizeFilter.filter(*surfPointsLessFlatDS);

  // DEBUG purpose
  // publish only the corner/flat points
  // sensor_msgs::PointCloud2 cornerPointsSharp2;
  // pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharp2);
  // cornerPointsSharp2.header.stamp = laserCloudIn2.header.stamp;
  // cornerPointsSharp2.header.frame_id = "/camera";
  // pubCornerPointsSharpPointer->publish(cornerPointsSharp2);

  // sensor_msgs::PointCloud2 cornerPointsLessSharp2;
  // pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharp2);
  // cornerPointsLessSharp2.header.stamp = laserCloudIn2.header.stamp;
  // cornerPointsLessSharp2.header.frame_id = "/camera";
  // pubCornerPointsLessSharpPointer->publish(cornerPointsLessSharp2);

  // sensor_msgs::PointCloud2 surfPointsFlat2;
  // pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
  // surfPointsFlat2.header.stamp = laserCloudIn2.header.stamp;
  // surfPointsFlat2.header.frame_id = "/camera";
  // pubSurfPointsFlatPointer->publish(surfPointsFlat2);

  // sensor_msgs::PointCloud2 surfPointsLessFlat2;
  // pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
  // surfPointsLessFlat2.header.stamp = laserCloudIn2.header.stamp;
  // surfPointsLessFlat2.header.frame_id = "/camera";
  // pubSurfPointsLessFlatPointer->publish(surfPointsLessFlat2);
  // end debug

  *laserCloudExtreCur += *cornerPointsSharp;
  *laserCloudExtreCur += *surfPointsFlat;
  *laserCloudLessExtreCur += *cornerPointsLessSharp;
  *laserCloudLessExtreCur += *surfPointsLessFlatDS;

  // ROS_INFO_STREAM("(corner sharp, surface flat, corner l, surface l, surface DS): (" << cornerPointsSharp->points.size() << ", " << surfPointsFlat->points.size() << ", " << cornerPointsLessSharp->points.size() << ", " <<  surfPointsLessFlat->points.size() << ", " << surfPointsLessFlatDS->points.size() << ")");

  laserCloudIn->clear();
  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();
  surfPointsLessFlatDS->clear();

  if (skipFrameCount >= skipFrameNum) {
    skipFrameCount = 0;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = imuPitchStart;
    imuTrans->points[0].y = imuYawStart;
    imuTrans->points[0].z = imuRollStart;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = imuPitchCur;
    imuTrans->points[1].y = imuYawCur;
    imuTrans->points[1].z = imuRollCur;
    imuTrans->points[1].v = 11;

    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    // debug
    // publish only the imu translation
    // sensor_msgs::PointCloud2 imuTrans2;
    // pcl::toROSMsg(*imuTrans, imuTrans2);
    // imuTrans2.header.stamp = laserCloudIn2.header.stamp;
    // imuTrans2.header.frame_id = "/camera";
    // pubImuTransPointer->publish(imuTrans2);
    // end debug

    sensor_msgs::PointCloud2 laserCloudExtreCur2;
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudExtreCur2);
    laserCloudExtreCur2.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudExtreCur2.header.frame_id = "/camera";
    pubLaserCloudExtreCurPointer->publish(laserCloudExtreCur2);
    imuTrans->clear();

    pubLaserCloudLastPointer->publish(laserCloudLast2); // this is the last registration before the new sweep

    ROS_INFO ("[REGISTRATION] (pc so far, pc this scan): (%d, %d)", laserCloudLast2.width, laserCloudExtreCur2.width);
  }
  skipFrameCount++;
}

void laserScanHandler(const sensor_msgs::LaserScanConstPtr& laserScanIn) {
  try {
      if(!listener_pointer->waitForTransform(
          laserScanIn->header.frame_id,
          "/base_link",
          laserScanIn->header.stamp + ros::Duration().fromSec(laserScanIn->ranges.size()*laserScanIn->time_increment),
          ros::Duration(15.0))) {
          // ROS_INFO_STREAM("SKIPPED " << laserScanIn->header.seq);
          return;
      }
  } catch(tf::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          return;
  }
   
  sensor_msgs::PointCloud2 cloud;
  lp.transformLaserScanToPointCloud( "/base_link", *laserScanIn, cloud, *listener_pointer);
  laserCloudHandler(cloud);
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  // get RPY
  double roll, pitch, yaw;

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);

  tf::Quaternion transformation_q;
  // -90 in y
  transformation_q.setRotation( tf::Vector3(0,1,0), -tfScalar(PI/2));

  tf::Quaternion parsed_orientation = orientation.operator*=(transformation_q);

  // geometry_msgs::PoseStamped imuVis;
  // // publish the imu quaternion
  // imuVis.pose.orientation.x = imuIn->orientation.x;
  // imuVis.pose.orientation.y = imuIn->orientation.y;
  // imuVis.pose.orientation.z = imuIn->orientation.z;
  // imuVis.pose.orientation.w = imuIn->orientation.w;
  // imuVis.pose.position.x = 0;
  // imuVis.pose.position.y = 0;
  // imuVis.pose.position.z = 0;

  
  // imuVis.header.stamp = imuIn->header.stamp;
  // imuVis.header.frame_id = "/camera_init_2";
  // pubImuVisPointer->publish(imuVis);

  
  tf::Matrix3x3(parsed_orientation).getRPY(roll, pitch, yaw);

  int imuPointerBack = imuPointerLast;
  imuPointerLast = (imuPointerLast + 1) % imuQueLength;
  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];

  // if two consecutive measurement were taken within 0.1 seconds
  if (timeDiff < 0.1) {

    // imuAccuRoll += timeDiff * imuIn->angular_velocity.x;
    // imuAccuPitch += timeDiff * imuIn->angular_velocity.y;
    // imuAccuYaw += timeDiff * imuIn->angular_velocity.z;

    // imuRoll[imuPointerLast] = roll;
    // imuPitch[imuPointerLast] = -pitch;
    // imuYaw[imuPointerLast] = -yaw;
    // imuRoll[imuPointerLast] = imuAccuRoll;
    // imuPitch[imuPointerLast] = -imuAccuPitch;
    // imuYaw[imuPointerLast] = -imuAccuYaw;

    imuRoll[imuPointerLast] = yaw;
    imuPitch[imuPointerLast] = roll;
    imuYaw[imuPointerLast] = -pitch;



////////////////////////////
  // tf::Quaternion input_quat;

  // input_quat.setRPY(imuRoll[imuPointerLast] , imuPitch[imuPointerLast] , imuYaw[imuPointerLast] );
  
  // // // ROS_INFO_STREAM(  input_quat.w() );

  // geometry_msgs::PoseStamped imuVis2;
  // // publish the imu quaternion
  // imuVis2.pose.orientation.x = input_quat.x();
  // imuVis2.pose.orientation.y = input_quat.y();
  // imuVis2.pose.orientation.z = input_quat.z();
  // imuVis2.pose.orientation.w = input_quat.w();
  // imuVis2.pose.position.x = 0;
  // imuVis2.pose.position.y = 0;
  // imuVis2.pose.position.z = 0;

  
  // imuVis2.header.stamp = imuIn->header.stamp;
  // imuVis2.header.frame_id = "/camera_init_2";
  // pubImuVisPointer2->publish(imuVis2);

////////////////////////////////

    // imuAccX[imuPointerLast] = -imuIn->linear_acceleration.y;
    // imuAccY[imuPointerLast] = -imuIn->linear_acceleration.z - 9.81;
    // imuAccZ[imuPointerLast] = imuIn->linear_acceleration.x;

    AccumulateIMUShift();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  tf::TransformListener listener;

  ros::Subscriber subLaserScan = nh.subscribe<sensor_msgs::LaserScan>("/lidar_scan", 4, laserScanHandler);

  // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
  //                                 ("/sync_scan_cloud_filtered", 4, laserCloudHandler);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> 
                                  ("/imu/data", 10, imuHandler);

  ros::Publisher pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2> 
                                         ("/laser_cloud_extre_cur", 2);

  // last registration before the new sweep
  ros::Publisher pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2> 
                                     ("/laser_cloud_last", 2);

  pubLaserCloudExtreCurPointer = &pubLaserCloudExtreCur;
  pubLaserCloudLastPointer = &pubLaserCloudLast;

  // debug purposes shows the corners and sharp areas of the current sweep of the lidar
  // ros::Publisher pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2> 
  //                                       ("/ms_cloud_sharp", 2);

  // ros::Publisher pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2> 
  //                                           ("/ms_cloud_less_sharp", 2);

  // ros::Publisher pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2> 
  //                                      ("/ms_cloud_flat", 2);

  // ros::Publisher pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2> 
  //                                          ("/ms_cloud_less_flat", 2);

  // ros::Publisher pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 10);

  // debug the laser angle
  // ros::Publisher pubLaserAngle = nh.advertise<std_msgs::Float32>("/laser_angle", 1000);

  // ros::Publisher pubFirstPoint = nh.advertise<sensor_msgs::PointCloud2>("/laser/first_point", 20);

  // ros::Publisher pubLastPoint = nh.advertise<sensor_msgs::PointCloud2>("/laser/last_point", 20);

  // debug imu visualiser
  // ros::Publisher pubImuVis = nh.advertise<geometry_msgs::PoseStamped> 
  //                                        ("/imu/q_pose", 20);
  // ros::Publisher pubImuVis2 = nh.advertise<geometry_msgs::PoseStamped> 
  //                                        ("/imu/q_pose_2", 20);

  // pubLaserAnglePointer = &pubLaserAngle;
  // pubCornerPointsSharpPointer = &pubCornerPointsSharp;
  // pubCornerPointsLessSharpPointer = &pubCornerPointsLessSharp;
  // pubSurfPointsFlatPointer = &pubSurfPointsFlat;
  // pubSurfPointsLessFlatPointer = &pubSurfPointsLessFlat;
  // pubImuTransPointer = &pubImuTrans;
  // pubFirstPointPointer = &pubFirstPoint;
  // pubLastPointPointer = &pubLastPoint;
  // pubImuVisPointer = &pubImuVis;
  // pubImuVisPointer2 = &pubImuVis2;

  listener_pointer = &listener;

  ros::spin();

  return 0;
}
