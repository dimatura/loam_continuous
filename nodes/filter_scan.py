#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import LaserScan, Imu

def callback(scanIn, imuIn):
  # Solve all of perception here...
  print("Test goes here")

rospy.init_node('filter_scan', anonymous=True)
scan_sub = message_filters.Subscriber('lidar_scan', LaserScan)
imu_sub = message_filters.Subscriber('imu/data', Imu)

ts = message_filters.TimeSynchronizer([scan_sub, imu_sub], 10)
ts.registerCallback(callback)
rospy.spin()
