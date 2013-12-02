#ifndef POINTCLOUDS_H
#define POINTCLOUDS_H

#include <vector>

class PointClouds {

private:
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> pointclouds;

public:
	void add_cloud(pcl::PointCloud<pcl::PointXYZRGBA>);
	void remove_cloud();
	bool isEmpty();
	void clear();
};

#endif
