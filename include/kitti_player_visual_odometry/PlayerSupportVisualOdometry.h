#ifndef _KITTI_PLAYER_SUPPORT_H_
#define _KITTI_PLAYER_SUPPORT_H_

#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#if !defined(_MSC_VER)
  #include <sys/select.h>
  #include <termios.h>
  #include <unistd.h>
#endif

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/progress.hpp>
#include <boost/timer.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#if BOOST_VERSION >= 104100
  #include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#if !defined(_MSC_VER)
  #include <sys/select.h>
#endif



using namespace std;

namespace PlayerSupportVisualOdometry
{

// Read camera calibration values and return by argument
int getCamCalibration(std::string dir_root, std::string camera_name, double* K, std::vector<double> & D, double *R, double* P);

/// Get timestamps fom the file
bool getTimestamp(string dir_timestamp,vector<ros::Time> &times);

bool get_poses(string dir_root,string sequence,vector<Eigen::Matrix3d> &rotations,vector<geometry_msgs::Point> &pts);


}

#endif // _KITTI_PLAYER_SUPPORT_H_
