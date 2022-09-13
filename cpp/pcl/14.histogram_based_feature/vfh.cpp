#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <string>
#include <sstream>


pcl::PointCloud<pcl::Normal>::Ptr surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // set target cloud being computed normal
  ne.setInputCloud(cloud);

  // create KdTree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  // set KdTree as the way of searching
  ne.setSearchMethod(tree);

  // argument for noraml
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // radius of range to search
  ne.setRadiusSearch(0.005);

  ne.compute(*cloud_normals);

  return cloud_normals;
}


pcl::PointCloud<pcl::VFHSignature308>::Ptr Extract_VFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // dst argument for normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());

  // VFH estimation class, and pass the input datatet+normals to it
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;

  cloud_normals = surface_normals(cloud);

  vfh.setInputCloud(cloud);
  vfh.setInputNormals(cloud_normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  vfh.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
  vfh.compute(*vfhs);
  return vfhs;
}


int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(argv[1], *cloud);


  std::stringstream Filename;
  std::string name = argv[1];
  name.erase(name.length() - 4);
  Filename << name << "_vfh.pcd";
  std::cout << Filename.str() << std::endl;
  pcl::io::savePCDFile(Filename.str(), *Extract_VFH(cloud));

  return 0;
}
