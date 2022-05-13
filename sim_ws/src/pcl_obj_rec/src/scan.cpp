#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataMapper.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "scanner");
	// Load the PLY model from a file.
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(argv[1]);
	reader->Update();

	// VTK is not exactly straightforward...
	vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	mapper->Update();

	vtkSmartPointer<vtkPolyData> object = mapper->GetInput();

	// Virtual scanner object.
	pcl::apps::RenderViewsTesselatedSphere render_views;
	render_views.addModelFromPolyData(object);
	// Pixel width of the rendering window, it directly affects the snapshot file size.
	render_views.setResolution(150);
	// Horizontal FoV of the virtual camera.
	render_views.setViewAngle(57.0f);
	// If true, the resulting clouds of the snapshots will be organized.
	render_views.setGenOrganized(true);
	// How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
	render_views.setTesselationLevel(1);
	// If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
	// This will affect the number of snapshots produced (if true, less will be made).
	// True: 42 for level 1, 162 for level 2, 642 for level 3...
	// False: 80 for level 1, 320 for level 2, 1280 for level 3...
	render_views.setUseVertices(true);
	// If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
	render_views.setComputeEntropies(true);

	render_views.generateViews();

	// Object for storing the rendered views.
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;
	// Object for storing the poses, as 4x4 transformation matrices.
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	// Object for storing the entropies (optional).
	std::vector<float> entropies;
	render_views.getViews(views);
	render_views.getPoses(poses);
	render_views.getEntropies(entropies);

    boost::filesystem::path dstFolder = "scans";
    if (boost::filesystem::exists(dstFolder)){
        boost::filesystem::remove_all(dstFolder);
        std::cout << "Previous folder removed." << std::endl;
    }
    boost::filesystem::create_directory(dstFolder);
    std::cout << "Scan folder created." << std::endl;

    
    for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::size_type i=0; i<views.size(); ++i){
        std::ostringstream oss;
        oss << "scans/cloud_scan_" << i << ".pcd";
        std::string name = oss.str();
        pcl::io::savePCDFileASCII(name, *views[i]);

        std::ofstream fout;
        fout.open("scans/poses.txt",std::ios::app);//Add writing at the end of the file
        fout << poses[i] << std::endl;//Wrap every time after writing a matrix
        fout.close();

        std::cout << "Saving Scan " << i << " with name: " << name << ". And pose:" << std::endl;
        std::cout << poses[i] << std::endl;
    }
    /*pcl::visualization::CloudViewer viewer(argv[1]);
    viewer.showCloud(views[0]);
	while (!viewer.wasStopped())
	{
		// Do nothing but wait.
	}*/
    return 0;

}