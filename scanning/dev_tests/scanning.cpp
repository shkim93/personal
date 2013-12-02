#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <math.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <math.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

/*  Structure for the extreme points in the 12 o'clock point cloud */
struct extremePoints{
	int pos_x0;
	int neg_x0; 
};

//finds the minimum z distance between camera and object
float findMinZ(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
  for(size_t i = 0; i< cloud->points.size(); i++){
     //approximates center point within x-y frame and returns z-value
     if(cloud->points[i].x >= -0.05 && cloud->points[i].y >= -0.05
        && cloud->points[i].x <= 0.05 && cloud->points[i].y <= 0.05 )
       return cloud->points[i].z;
  }
  return -100.0; //this signifies an error
}

//Remove floor points
void removeFloorPoints( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr my_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud ) {
  float maxY = -9999999;

  for( size_t i=0; i < my_cloud->points.size(); ++i )
  {
    if( my_cloud->points[i].y > maxY ) {
      maxY = my_cloud->points[i].y;
    }
  }
  std::cout << "Max Y = " << maxY << std::endl;

  pcl::PassThrough<pcl::PointXYZRGBA> removeFloor;
  removeFloor.setInputCloud(my_cloud);
  removeFloor.setFilterFieldName("y");
  removeFloor.setFilterLimits(-9999999, maxY - 0.2);
  removeFloor.filter(*filtered_cloud);
}

//sets up parameters for filter then filters result
void filterDimension(pcl::PassThrough<pcl::PointXYZRGBA> f, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud, std::string field, float limit) {
  f.setInputCloud (input_cloud);
  f.setFilterFieldName (field);
  f.setFilterLimits (-limit, limit);
  f.filter (*filtered_cloud);
}

//filter function
void filterCloud(pcl::PointCloud<pcl::PointXYZRGBA>Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>Ptr filtered_cloud) {

  //Set up limits for filter for isolating human body from pointcloud
  //*Note: Kinect should be set up no more than 1.75m above ground level
  //*Note: all dimensions in meters

  float x; //= 1.3
  float y; //= 1.3
  float z; //= between 0.8 and 1
  float minZ;
  std::cout << "Enter a x-limit for the filter: ";
  std::cin >> x;
  std::cout << "Enter height of kinect: ";
  std::cin >> y;
  std::cout << "Enter a depth value to add to Min z-value: ";
  std::cin >> z;
  minZ = findMinZ(input_cloud);
  std::cout << "Min Z = " << minZ << ", X-limit = " << x << ", Y-limit = " << y << std::endl;
  z = z + minZ;

  //Set up filters for each of the dimensions
  pcl::PassThrough<pcl::PointXYZRGBA> passX;
  filterDimension(passX, input_cloud, filtered_cloud, "x", x);
  pcl::PassThrough<pcl::PointXYZRGBA> passY;
  filterDimension(passY, filtered_cloud, filtered_cloud, "y", y);
  pcl::PassThrough<pcl::PointXYZRGBA> passZ;
  filterDimension(passZ, filtered_cloud, filtered_cloud, "z", z);

  removeFloorPoints(filtered_cloud, filtered_cloud);

  // Filtering input scan to roughly 10% of original size to increase speed of registration
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final_filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.01, 0.01, 0.01); //base values were all set to 0.2 for 100,000p point cloud from tutorial
  approximate_voxel_filter.setInputCloud (filtered_cloud);
  approximate_voxel_filter.filter (*final_filtered_cloud);
  std::cout << "Filtered cloud contains " << final_filtered_cloud->size ()
            << " data points from " << filtered_cloud << std::endl;

  pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> filterOutliers;
  filterOutliers.setInputCloud (final_filtered_cloud);
  filterOutliers.setRadiusSearch (.02);
  filterOutliers.setMinNeighborsInRadius (5);
  filterOutliers.filter (*filtered_cloud);
}

Eigen::Matrix4f getRyMatrix(float theta, float x, float z){
	float PI = 3.1415926535; //Fill out correct value here
	float theta_rad = (theta * PI) / 180.0;
	Eigen::Matrix4f rotMat;
	rotMat << cos(theta_rad), 0,  sin(theta_rad), 0, 0, 1, 0, 0, -1*sin(theta_rad), 0, cos(theta_rad), 0, x, 0, z, 1;
	std::cout << "z: " << z << " x: " << x << std::endl;
  return rotMat;
}

/*
 * Find the most positive X point for the 4 o'clock point cloud 
 */
int findPosX(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_4o){

	size_t max_index = 0; // index of point with most positive X
	float current_max = cloud_4o->points[0].x; // value of current most positive X point
	
	// bubble out the maximum X value
	for(size_t i = 1; i < cloud_4o->points.size(); i++){
		if(cloud_4o->points[i].y && cloud_4o->points[i].x > current_max){
			current_max = cloud_4o->points[i].x;
			max_index = i;
		}
	}
	
	// store most positive point
	pcl::PointXYZRGBA pos_x = cloud_4o->points[max_index]; 
	return max_index;
}

/*
 * Find the most negative X point for the 8 o'clock point cloud
 */
int findNegX(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_8o){

	size_t min_index = 0; // index of point with most positive X
	float current_min = cloud_8o->points[0].x; // value of current most positive X point
	
	// bubble out the minimum X value
	for(size_t i = 1; i < cloud_8o->points.size(); i++){
		if(cloud_8o->points[i].y < 0 && cloud_8o->points[i].x < current_min){
			current_min = cloud_8o->points[i].x;
			min_index = i;
		}
	}
	return min_index;
}

/*
 * Find the most negative X and most positive X points for the 12 o'clock point cloud
 */
extremePoints findCriticalPoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_12o){
	
	extremePoints x0s;
	
	x0s.pos_x0 = findPosX(cloud_12o);
	x0s.neg_x0 = findNegX(cloud_12o);
	
	return x0s;
}

/*
//creates event listener that saves cloud when spacebar is pressed
void keyboardEventOccurred(const visualization::KeyboardEvent& event,
    void* nothing)
{
	//press 't' to capture 12:00 cloud
    if (event.getKeySym() == "t" && event.keyDown())
        get_12o = true;
	//press 'h' to capture 4:00 cloud
    if (event.getKeySym() == "h" && event.keyDown())
        get_4o = true;
	//press 'f' to capture 8:00 cloud
    if (event.getKeySym() == "f" && event.keyDown())
        get_8o = true;
}

boost::shared_ptr<visualization::CloudViewer> createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v
        (new visualization::CloudViewer("3D Viewer"));
    v->registerKeyboardCallback(keyboardEventOccurred);
    
    return(v);
}

void
grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
    if (! viewer->wasStopped())
        viewer->showCloud(cloud);
        
    if (saveCloud)
    {
        stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        string filename = stream.str();
        if (io::savePCDFile(filename, *cloud, true) == 0)
        {
            filesSaved++;
            cout << "Saved " << filename << "." << endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());
        
        saveCloud = false;
    }
}
*/

int main (int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_4o_orig (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_8o_orig (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_12o_orig (new pcl::PointCloud<pcl::PointXYZRGBA>);

  int index_4o;
  int index_8o;
  extremePoints index_12o; 

  if ( argc > 4 ) {
    std::cout << "usage: " << argv[0] << " <filename.pcd> <filename.pcd>\n" << std::endl;
  }
  else {
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud_4o_orig) == -1) {
      PCL_ERROR ("Could not read file\n");
      return (-1);
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[2], *cloud_8o_orig) == -1) {
      PCL_ERROR ("Could not read file\n");
      return (-1);
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[3], *cloud_12o_orig) == -1) {
      PCL_ERROR ("Could not read file\n");
      return (-1);
    }
  }

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_4o = (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_8o = (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_12o = (new pcl::PointCloud<pcl::PointXYZRGBA>);

	filterCloud(cloud_4o_orig, cloud_4o);
	filterCloud(cloud_8o_orig, cloud_8o);
	filterCloud(cloud_12o_orig, cloud_12o);

  index_4o = findPosX(cloud_4o);
  index_8o = findNegX(cloud_8o);
  index_12o = findCriticalPoints(cloud_12o);

  Eigen::Matrix4f rotate_4o = getRyMatrix(-120,0,0);
  Eigen::Matrix4f rotate_8o = getRyMatrix(120,0,0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_4o_final (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_8o_final (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::transformPointCloud(*cloud_4o, *cloud_4o_final, rotate_4o);
  pcl::transformPointCloud(*cloud_8o, *cloud_8o_final, rotate_8o);

  float trans_4o_x = cloud_12o->points[index_12o.neg_x0].x-cloud_4o_final->points[index_4o].x;
  float trans_4o_z = cloud_12o->points[index_12o.neg_x0].z-cloud_4o_final->points[index_4o].z;

  float trans_8o_x = cloud_12o->points[index_12o.pos_x0].x-cloud_8o_final->points[index_8o].x;
  float trans_8o_z = cloud_12o->points[index_12o.pos_x0].z-cloud_8o_final->points[index_8o].z;

  //Eigen::Matrix4f translate_4o = getRyMatrix(0, 100,100);//trans_4o_x, trans_4o_z);
  //Eigen::Matrix4f translate_8o = getRyMatrix(0, 100,100);//trans_8o_x, trans_8o_z);

  //pcl::transformPointCloud(*cloud_4o_temp, *cloud_4o, translate_4o);
  //pcl::transformPointCloud(*cloud_8o_temp, *cloud_8o, translate_8o);

  for(size_t i = 0; i< cloud_4o_final->points.size(); i++){
    cloud_4o_final->points[i].x+=trans_4o_x; 
    cloud_4o_final->points[i].z+=trans_4o_z; 
  }
  for(size_t i = 0; i< cloud_8o_final->points.size(); i++){
    cloud_8o_final->points[i].x+=trans_8o_x; 
    cloud_8o_final->points[i].z+=trans_8o_z; 
  }

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // 4o
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
  color_4o (cloud_4o_final, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZRGBA> (cloud_4o_final, color_4o, "output cloud 1");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud 1");

  // 8o
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
  color_8o (cloud_8o_final, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZRGBA> (cloud_8o_final, color_8o, "output cloud 2");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud 2");

  // 12o
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>
  color_12o (cloud_12o, 255, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZRGBA> (cloud_12o, color_12o, "output cloud 3");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud 3");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0);
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


	return 0;
}




