#ifndef THREAD_ASYNC_H
#define THREAD_ASYNC_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <thread>

#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>



class ThreadClass {
public:
  ThreadClass();
  virtual ~ThreadClass();

private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // reate Assync spiner
  ros::AsyncSpinner spinner;

  // PUBLISHERS
  ros::Subscriber sub;
  //ros::Subscriber odom_sub;

  void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
  //void ODOM_callback(const nav_msgs::Odometry::ConstPtr &msg);
  pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
  pcl::visualization::PCLVisualizer::Ptr viewer;
};

#endif // THREAD_ASYNC_H