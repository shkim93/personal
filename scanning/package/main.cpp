#include "trap.h"

#include <iostream>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

boost::shared_ptr<pcl::visualization::PCLViewer> grabberViewer;
boost::shared_ptr<pcl::visualization::PCLViewer> modelViewer;
pcl::Grabber* kinectGrabber;
unsigned int filesSaved = 0;
bool saveCloud(false);
Trap trapper;

void grabberCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	if (! grabberViewer->wasStopped())
	  grabberViewer->showCloud(cloud);
        
  if (saveCloud)
  {
		trapper.addCloud(cloud);        
	  saveCloud = false;
  }
}

void keyboardEventOccurred1(const pcl::visualization::KeyboardEvent& event,
     void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
}

void keyboardEventOccurred2(const pcl::visualization::KeyboardEvent& event,
     void* nothing)
{
    if (event.getKeySym() == "c" && event.keyDown())
        grabberViewer.close();
}

boost::shared_ptr<pcl::visualization::PCLViewer> createGrabberViewer()
{
	boost::shared_ptr<pcl::visualization::PCLViewer> v
  	(new pcl::visualization::PCLViewer("Kinect Grabber"));

  v->registerKeyboardCallback(keyboardEventOccurred1);
  v->registerKeyboardCallback(keyboardEventOccurred2);

  return(v);
}

int main(int argc, char** argv) {

	std:cout << "Welcome to the BodySim model building application" << std:endl;
	std:cout << std:endl;

	trapper = new Trap();

	kinectGrabber = new pcl::OpenNIGrabber();
 	if (kinectGrabber == 0)
    return false;
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f 
		= boost::bind(&grabberCallback, _1);

  kinectGrabber->registerCallback(f);
    
  grabberViewer = createGrabberViewer();
  
	std:cout << "Press 'c' to exit grabber" << std::endl;

  kinectGrabber->start();
    
  while (!grabberViewer->wasStopped())
    boost::this_thread::sleep(boost::posix_time::seconds(1));

  kinectGrabber->stop();


	trapper.showClouds();
	
	return 0;
}
