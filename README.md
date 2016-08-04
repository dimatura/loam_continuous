# LOAM Multisense
Lidar Odometry and Mapping (LOAM) ported for Multisense SL.
Tested on ROS Indigo, on a laptop with 2.9GHz quad cores and 16gb memory (consumes 2 cores).

### How to use
Here we assume you have a Catkin workspace under ~/ros_workspace/catkin_ws.

(1) gitclone the package into your "src" folder.
+ **cd ~/ros_workspace/catkin_ws/src**
+ **git clone https://github.com/ipab-slmc/loam_continuous**

(2) compile the package
+ **cd ~/ros_workspace/catkin_ws**
+ **catkin_make**

(4) run the package and rosbag file
+ 1) in 1st terminal:
+ **roslaunch loam_multisense loam_multisense.launch**
+ 2) in 2nd terminal:
+ **rosbay play <bag_name>.bag**


### Authors' Original README

Lidar Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar, and optionally an IMU. The program contains four nodes. The “scanRegistration” node stacks laser scans within a sweep and publishes them as point cloud. The “laserOdometry” node estimates motion of the lidar between two sweeps, at a higher frame rate. The node corrects distortion in the point cloud from motion of the lidar. The “laserMapping” node takes the output of “laser_odometry” and incrementally builds a map. It also computes pose of the lidar on the map, at a lower frame rate. The state estimation of the lidar is combination of the outputs from “laserOdometry” and “laserMapping”, integrated in the “transformMaintenance” node. 

The program is tested on ROS Fuerte, on a laptop computer with 2.5 GHz quad cores and 6 Gib memory (the program consumes two cores). This version uses a lidar that spins continuously.

Wiki Webpage: http://wiki.ros.org/loam_continuous

Another version of the program that uses back and forth spin is available at

Wiki Webpage: http://wiki.ros.org/loam_back_and_forth

GitHub Code: https://github.com/jizhang-cmu/loam_back_and_forth.git

How to use:

1) Download the program file to a ROS directory, unpack the file and rename the folder to “loam_continuous” (GitHub may add "-xxx" to the end of the folder name). Go to the folder and “rosmake”, then “roslunch loam_continuous.launch”. The launch file should start the program (with four nodes) and rviz.

2) Download datasets from the following website. Make sure the data files are for continuous spin (not back and forth spin). Play the data files with “rosbag play data_file_name.bag”. Note that if a slow computer is used, users can try to play the data files at a lower speed, e.g. “rosbag play data_file_name.bag -r 0.5” plays the data file at half speed.

Datasets can be downloaded at: http://www.frc.ri.cmu.edu/~jizhang03/Datasets/
