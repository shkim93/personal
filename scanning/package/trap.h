#ifndef TRAP_H
#define TRAP_H

#include <vector>

class Trap {

private:
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> pointclouds;

public:
	Trap();
	void addCloud();
	void removeCloud();
	

};

#endif



