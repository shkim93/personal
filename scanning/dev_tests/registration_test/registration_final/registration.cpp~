#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

//code taken from http://pointclouds.org/documentation/tutorials/normal_estimation.php
void getNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  ne.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*normals);
}

//code grabbed from http://pointclouds.org/documentation/tutorials/fpfh_estimation.php
void getFeatures(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs) {

  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (cloud);
  fpfh.setInputNormals (normals);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);

  fpfh.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (0.05);

  // Compute the features
  fpfh.compute (*fpfhs);

  // fpfhs->points.size () should have the same size as the input cloud->points.size ()*
}

/* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
  * correspondences between two sets of local features
  * Inputs:
  *  source_points
  *    The "source" points, i.e., the points that must be transformed to align with the target point cloud
  *  source_descriptors
  *    The local descriptors for each source point
  *  target_points
  *    The "target" points, i.e., the points to which the source point cloud will be aligned
  *  target_descriptors
  *    The local descriptors for each target point
  *  min_sample_distance
  *    The minimum distance between any two random samples
  *  max_correspondence_distance
  *    The
  *  nr_interations
  *    The number of RANSAC iterations to perform
  * Return: A transformation matrix that will roughly align the points in source to the points in target
  */
Eigen::Matrix4f computeInitialAlignment (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_descriptors, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_descriptors, float min_sample_distance, float max_correspondence_distance, int nr_iterations) {

  pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::FPFHSignature33> sac_ia;
  sac_ia.setMinSampleDistance (min_sample_distance);
  sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
  sac_ia.setMaximumIterations (nr_iterations);
 
  sac_ia.setInputSource (input_cloud);
  sac_ia.setSourceFeatures (input_descriptors);
 
  sac_ia.setInputTarget (target_cloud);
  sac_ia.setTargetFeatures (target_descriptors);
 
  pcl::PointCloud<pcl::PointXYZRGBA> registration_output;
  sac_ia.align (registration_output);
 
  return (sac_ia.getFinalTransformation ());
}

/* Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
  * starting with an intial guess
  * Inputs:
  *  source_points
  *    The "source" points, i.e., the points that must be transformed to align with the target point cloud
  *  target_points
  *    The "target" points, i.e., the points to which the source point cloud will be aligned
  *  intial_alignment
  *    An initial estimate of the transformation matrix that aligns the source points to the target points
  *  max_correspondence_distance
  *    A threshold on the distance between any two corresponding points. Any corresponding points that are further
  *    apart than this threshold will be ignored when computing the source-to-target transformation
  *  outlier_rejection_threshold
  *    A threshold used to define outliers during RANSAC outlier rejection
  *  transformation_epsilon
  *    The smallest iterative transformation allowed before the algorithm is considered to have converged
  *  max_iterations
  *    The maximum number of ICP iterations to perform
  * Return: A transformation matrix that will precisely align the points in source to the points in target
  */
Eigen::Matrix4f refineAlignment ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, const  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud, Eigen::Matrix4f initial_alignment, float max_correspondence_distance, float outlier_rejection_threshold, float transformation_epsilon, float max_iterations) {
 
  pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
  icp.setMaxCorrespondenceDistance (max_correspondence_distance);
  icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
  icp.setTransformationEpsilon (transformation_epsilon);
  icp.setMaximumIterations (max_iterations);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::transformPointCloud (*input_cloud, *input_cloud_transformed, initial_alignment);
 
  icp.setInputSource (input_cloud_transformed);
  icp.setInputTarget (target_cloud);
 
  pcl::PointCloud<pcl::PointXYZRGBA> registration_output;
  icp.align (registration_output);

  return (icp.getFinalTransformation () * initial_alignment);
}

int main(int argc, char** argv) {

  //point clouds in question
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

  //*****output of getNormals function
  pcl::PointCloud<pcl::Normal>::Ptr input_cloud_norm (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr target_cloud_norm (new pcl::PointCloud<pcl::Normal>);

  //*****output of getFeatures function
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  //*****output of computeInitialAlignment
  Eigen::Matrix4f initTrans;

  //*****output of refineAlignment
  Eigen::Matrix4f refTrans;

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
  
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud, indices);
  std::vector<int> indices0;
  pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices0);

  getNormals(input_cloud, input_cloud_norm);
  getNormals(target_cloud, target_cloud_norm);

  getFeatures(input_cloud, input_cloud_norm, input_fpfhs);
  getFeatures(target_cloud, target_cloud_norm, target_fpfhs);

  initTrans = computeInitialAlignment(input_cloud, input_fpfhs, target_cloud, target_fpfhs, 0.05f, 0.1f*0.1f, 1);
  //refTrans = refineAlignment(input_cloud, target_cloud, initTrans, 0.05, 0.1, 0.01, 30);

  refTrans = refineAlignment(input_cloud, target_cloud, initTrans, 0.05, 0.05, 1e-6, 35);

  pcl::transformPointCloud(*input_cloud, *output_cloud, refTrans);

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
