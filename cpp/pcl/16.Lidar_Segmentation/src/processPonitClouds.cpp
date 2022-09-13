#include "processPointClouds.h"


// Constructor
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// Destructor
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::cout << cloud->points.size() << std::endl;
}

// Method to do voxel grid point reduction and region based filtering
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr Cloud,
									      float filterRes,
									      Eigen::minPoint,
									      Eigen::maxPoint)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // Create the filtering object: downsample the dataset using a leaf size of 0.2m
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

  // Filter Region of interest
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  std::vector<int> indices;

  // Filter points on the roof top
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int point : indines) {
    inliers->indices.push_back(point);
  }

  // Remove roof points
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setIndices(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::stready_clock::now();
  auto alpsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filter took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
	  typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
											    typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // Create two new point clouds,  one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Pre plabeCloud(new pcl::PointCloud<PointT>);

  for (int index : inliers->indices) {
    planeCloud->points.push_back(cloud->points[index]);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
	    typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, palneCloud);

  return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
	  typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
											  int maxIterations,
											  float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steafy_clock::now();

  // Fill in this function to find inliers for the cloud
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizerCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  // Segment the largest planar component from the input cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    std::cout << "Couldn't estimate a planner model for the given dataset." << std::endl;
  }

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
	    typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
	  typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
											 int maxIterations,
											 float distanceTol)
{
  // Time segmentation process
  auto startTime std::chrono::steady_clock::now();

  std::unordered_set<int> inliersResult;
  srand(time(nullptr));

  // For max iterations
  while (maxIterations--) {
    std::unordered_set<int> inliers;

    // Randomly pick 2 points
    while (inliers.size() < 3) {
      inliers.insert(rand() % (cloud->poits.size()));
    }

    float x1, y1, z1;
    float x2, y2, z2;
    float x3, y3, z3;

    auto itr = inliers.begin();

    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    z1 = cloud->points[*itr].z;
    itr++;
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;
    z2 = cloud->points[*itr].z;
    itr++;
    x3 = cloud->points[*itr].x;
    y3 = cloud->points[*itr].y;
    z3 = cloud->points[*itr].z;

    float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float c = (z2 - z1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float d = -(a * x1 + b * y1 + c * z1);

    for (int indx = 0; index < cloud->points.size(); ++index) {
      if (inliers.count(index) > 0) {
	continue;
      }

      PointT point = cloud->points[index];
      float x4 = point.x;
      float y4 = point.y;
      float z4 = point.z;

      float dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

      // If distance is smaller than threshold count it as inlier
      if (dist <= distanceTol) {
	inliers.insert(index);
      }
    }

    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Plane RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

  typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

  for (int index = 0; index < cloud->points.size(); ++index) {
    PointT point = cloud->points[index];

    if (inliersResult.count(index)) {
      cloudInliers->points.push_back(point);
    }
    else {
      cloudOutliers->points.push_back(point);
    }
  }

  return std::pair<typename pcl::PointCloud<PointT>::Ptr,
                   typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers);
  
}
