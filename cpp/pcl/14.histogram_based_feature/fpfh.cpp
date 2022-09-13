#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <sstream>
#include <string>


pcl::PointCloud<pcl::Normal>::Ptr sruface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.05);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  ne.compute(*cloud_normals);

  return cloud_normals;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr Extract_FPFH(pcl::PointCloud<PointXYZ>::Ptr cloud)
{
  // dst
  pcl::FPFH
}
