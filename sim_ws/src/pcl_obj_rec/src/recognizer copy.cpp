#include "pcl_obj_rec/async_threads.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_config.h>

ThreadClass::ThreadClass() : nh_(""), spinner(1) {

  ROS_INFO("Init Started");

  spinner.start();

  sub = nh_.subscribe("input", 10,
                                   &ThreadClass::cloud_cb, this);

  
  
  ROS_INFO("Init Finished");
}

ThreadClass::~ThreadClass() { ros::shutdown(); }

void ThreadClass::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

  ROS_INFO("Cloud received");
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  
  ThreadClass::viewer = ThreadClass::rgbVis(temp_cloud);
  ThreadClass::viewer->spinOnce (100);
}

pcl::visualization::PCLVisualizer::Ptr ThreadClass::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
 {
   // --------------------------------------------
   // -----Open 3D viewer and add point cloud-----
   // --------------------------------------------
   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
   viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
   return (viewer);
 }

int main(int argc, char **argv) {
  // Init ROS node
  ros::init(argc, argv, "recognizer");
  ThreadClass thread_object;

  ros::waitForShutdown();

  return 0;
}

/*
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_config.h>
#include <pcl/visualization/cloud_viewer.h>


ros::Publisher pub;

void 
cloud_cb (const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);


  //pcl_conversions::toPCL(*input, *cloud);
  //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud(input); 
  //pcl_conversions::toPCL(*input, *cloud);
  pcl::visualization::CloudViewer viewer("TESTE");

	viewer.showCloud(temp_cloud);





  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "recognizer");
  ros::NodeHandle nh;
  std::cout << PCL_VERSION << std::endl;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}

*/