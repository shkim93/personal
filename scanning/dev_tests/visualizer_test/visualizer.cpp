#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

Eigen::Matrix4f getRyMatrix(float theta){
	float PI = 3.1415926535; //Fill out correct value here
	float theta_rad = (theta * PI) / 180.0;
	
	Eigen::Matrix4f rotMat;
	rotMat << cos(theta_rad), 0,  sin(theta_rad), 0, 0, 1, 0, 0, -1*sin(theta_rad), 0, cos(theta_rad), 0, 0, 0, 0, 1;
	
	return rotMat;
}


int main(int argc, char** argv) {

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

  //program accepts 2 .pcd files. *change to 3 later
  if ( argc > 3 ) {
    std::cout << "usage: " << argv[0] << " <filename.pcd> <filename.pcd>\n" << std::endl;
  }
  else {
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *input_cloud) == -1) {
      PCL_ERROR ("Could not read file\n");
      return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from " << argv[1] << std::endl;

    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[2], *target_cloud) == -1) {
      PCL_ERROR ("Could not read file\n");
      return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from " << argv[2] << std::endl;
  }

  Eigen::Matrix4f transform_guess = getRyMatrix(120);

  pcl::transformPointCloud(*input_cloud, *output_cloud, transform_guess);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
  output_color (output_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZRGBA> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
  target_color (target_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZRGBA> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0);
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
