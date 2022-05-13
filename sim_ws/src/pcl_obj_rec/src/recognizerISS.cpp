#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/iss_3d.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class cloudHandler
{
public:
    cloudHandler():viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("/camera/depth_registered/points", 10, &cloudHandler::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB,this);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg (input, cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);

        cloudPtr = cloud.makeShared();

	

	    // ISS keypoint detector object.
	    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
	    detector.setInputCloud(cloudPtr);
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	    detector.setSearchMethod(kdtree);
	    double resolution = computeCloudResolution(cloudPtr);
	    // Set the radius of the spherical neighborhood used to compute the scatter matrix.
	    detector.setSalientRadius(6 * resolution);
	    // Set the radius for the application of the non maxima supression algorithm.
	    detector.setNonMaxRadius(4 * resolution);
	    // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	    detector.setMinNeighbors(5);
	    // Set the upper bound on the ratio between the second and the first eigenvalue.
	    detector.setThreshold21(0.975);
	    // Set the upper bound on the ratio between the third and the second eigenvalue.
	    detector.setThreshold32(0.975);
	    // Set the number of prpcessing threads to use. 0 sets it to automatic.
	    detector.setNumberOfThreads(4);
    
	    detector.compute(*keypoints);
        std::cout << "Found " << keypoints->size() << " keypoints." << std::endl;
        //viewer.showCloud(cloudPtr);
        viewer.showCloud(keypoints);
    }
     
    void timerCB(const ros::TimerEvent&)
    {
        if(viewer.wasStopped())
            {
                ros::shutdown();
            }
    }
    double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
    {
    	double resolution = 0.0;
    	int numberOfPoints = 0;
    	int nres;
    	std::vector<int> indices(2);
    	std::vector<float> squaredDistances(2);
    	pcl::search::KdTree<pcl::PointXYZ> tree;
    	tree.setInputCloud(cloud);

    	for (size_t i = 0; i < cloud->size(); ++i)
    	{
    		if (! std::isfinite((*cloud)[i].x))
    			continue;

    		// Considering the second neighbor since the first is the point itself.
    		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
    		if (nres == 2)
    		{
    			resolution += sqrt(squaredDistances[1]);
    			++numberOfPoints;
    		}
    	}
    	if (numberOfPoints != 0)
    		resolution /= numberOfPoints;

    	return resolution;
    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_visualize");
    cloudHandler handler;
    ros::spin();
    return 0;
}