#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

//finds the minimum z distance between camera and object
float findMinZ(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
  for(size_t i = 0; i< cloud->points.size(); i++){
     //finds point relatively close to center of camera's view and returns z-value
     if(cloud->points[i].x >= -0.1 && cloud->points[i].y >= -0.1\
        && cloud->points[i].x <= 0.1 && cloud->points[i].y <= 0.1 )
       return cloud->points[i].z;
  }
  return -100.0; //this signifies an error

}

int
 main (int argc, char** argv)
{
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passZ_filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passX_filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

//reads in .pcd file from command line
  if ( argc > 3 ) {
    std::cout << "usage: " << argv[0] << " <filename.pcd> <filename.pcd>\n" << std::endl;
  }
  else {
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *input_cloud) == -1) {
      PCL_ERROR ("Could not read file\n");
      return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from " << argv[1] << std::endl;
  }
  //Create the depth filtering object
  pcl::PassThrough<pcl::PointXYZRGBA> passZ;
  passZ.setInputCloud (input_cloud);
  passZ.setFilterFieldName ("z");

  float z = findMinZ(input_cloud);

  std::cout << "Min Z = " << z << std::endl;
  //set filter limit to 0.5 meters past the Min z-value to capture object && filter out background
  passZ.setFilterLimits (0.0, z+0.5);
  passZ.filter (*passZ_filtered_cloud);

  //Create the filter object to remove sides
  pcl::PassThrough<pcl::PointXYZRGBA> passX;
  passX.setInputCloud (passZ_filtered_cloud);
  passX.setFilterFieldName ("x");

  passX.setFilterLimits (-0.5, 0.5);
  passX.filter (*passX_filtered_cloud);

  pcl::io::savePCDFileASCII ("filtered_cloud.pcd", *passX_filtered_cloud);

  return (0);
}
