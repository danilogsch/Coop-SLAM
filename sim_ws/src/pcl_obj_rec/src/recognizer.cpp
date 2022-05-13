#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/iss_3d.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.03f);
float scene_ss_ (2.0f);
float rf_rad_ (0.015f);
float descr_rad_ (1.0f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

class cloudHandler
{
public:
    cloudHandler():viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, &cloudHandler::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB,this);

		
    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
		{
		  std::cout << "Error loading model cloud." << std::endl;
		  //showHelp (argv[0]);
		  //return (-1);
		}

		if (use_cloud_resolution_)
		{
		  float resolution = static_cast<float> (computeCloudResolution (model));
		  if (resolution != 0.0f)
		  {
		    model_ss_   *= resolution;
		    scene_ss_   *= resolution;
		    rf_rad_     *= resolution;
		    descr_rad_  *= resolution;
		    cg_size_    *= resolution;

		  std::cout << "Model resolution:       " << resolution << std::endl;
		  std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		  std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		  std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		  std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		  std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
		  }
    }

		//pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
		norm_est.setKSearch (10);
		norm_est.setInputCloud (model);
		norm_est.compute (*model_normals);

		//pcl::UniformSampling<PointType> uniform_sampling;
		uniform_sampling.setInputCloud (model);
		uniform_sampling.setRadiusSearch (model_ss_);
		uniform_sampling.filter (*model_keypoints);

		//pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
		descr_est.setRadiusSearch (descr_rad_);	

		descr_est.setInputCloud (model_keypoints);
		descr_est.setInputNormals (model_normals);
		descr_est.setSearchSurface (model);
		descr_est.compute (*model_descriptors);


		std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;


		std::cout << "Initialization complete." << std::endl;

	}
    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        pcl::fromROSMsg (input, cloud);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	      //pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
        scene = cloud.makeShared();

        norm_est.setInputCloud (scene);
		    norm_est.compute (*scene_normals);

		    uniform_sampling.setInputCloud (scene);
		    uniform_sampling.setRadiusSearch (scene_ss_);
		    uniform_sampling.filter (*scene_keypoints);

        descr_est.setNumberOfThreads(4);
        descr_est.setInputCloud (scene_keypoints);
		    descr_est.setInputNormals (scene_normals);
		    descr_est.setSearchSurface (scene);
		    descr_est.compute (*scene_descriptors);

        std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

        match_search.setInputCloud (model_descriptors);
        pcl::CorrespondencesPtr model_scene_corrs {new pcl::Correspondences ()};

        //for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
        //{
        //  std::vector<int> neigh_indices (1);
        //  std::vector<float> neigh_sqr_dists (1);
        //  if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        //  {
        //    continue;
        //  }
        //  int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        //  if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        //  {
        //    pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        //    model_scene_corrs->push_back (corr);
        //  }
        //}
        std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
        viewer.showCloud(scene);


    }
     
    void timerCB(const ros::TimerEvent&)
    {
        if(viewer.wasStopped())
            {
                ros::shutdown();
            }
    }
    double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
	  {
	    double res = 0.0;
	    int n_points = 0;
	    int nres;
	    std::vector<int> indices (2);
	    std::vector<float> sqr_distances (2);
	    pcl::search::KdTree<PointType> tree;
	    tree.setInputCloud (cloud);

	    for (std::size_t i = 0; i < cloud->size (); ++i)
	    {
	      if (! std::isfinite ((*cloud)[i].x))
	      {
	        continue;
	      }
	      //Considering the second neighbor since the first is the point itself.
	      nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
	      if (nres == 2)
	      {
	        res += sqrt (sqr_distances[1]);
	        ++n_points;
	      }
	    }
	    if (n_points != 0)
	    {
	      res /= n_points;
	    }
	    return res;
	  }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
		pcl::PointCloud<PointType>::Ptr model {new pcl::PointCloud<PointType> ()};
		pcl::PointCloud<PointType>::Ptr model_keypoints {new pcl::PointCloud<PointType> ()};
    pcl::PointCloud<PointType>::Ptr scene {new pcl::PointCloud<PointType> ()}; 		
		pcl::PointCloud<PointType>::Ptr scene_keypoints {new pcl::PointCloud<PointType> ()};
		pcl::PointCloud<NormalType>::Ptr model_normals {new pcl::PointCloud<NormalType> ()};
		pcl::PointCloud<NormalType>::Ptr scene_normals {new pcl::PointCloud<NormalType> ()};
		pcl::PointCloud<DescriptorType>::Ptr model_descriptors {new pcl::PointCloud<DescriptorType> ()};
		pcl::PointCloud<DescriptorType>::Ptr scene_descriptors {new pcl::PointCloud<DescriptorType> ()};

    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;

    pcl::UniformSampling<PointType> uniform_sampling;

    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;

    //pcl::CorrespondencesPtr model_scene_corrs {new pcl::Correspondences ()};
    pcl::KdTreeFLANN<DescriptorType> match_search;

};


void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}

void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 1)
  {
    std::cout << "Model filename missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];

  //Program behavior
  if (pcl::console::find_switch (argc, argv, "-k"))
  {
    show_keypoints_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    show_correspondences_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    use_cloud_resolution_ = true;
  }

  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("Hough") == 0)
    {
      use_hough_ = true;
    }else if (used_algorithm.compare ("GC") == 0)
    {
      use_hough_ = false;
    }
    else
    {
      std::cout << "Wrong algorithm name.\n";
      //showHelp (argv[0]);
      //exit (-1);
    }
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_visualize");
	  parseCommandLine (argc, argv);

    cloudHandler handler;
    ros::spin();
    return 0;
}