#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
    srand((unsigned int)time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // generate point cloud data
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    float resolusion = 128.0f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolusion);

    octree.setInputCloud(cloud);

    octree.addPointsFromInputCloud();

    pcl::PointXYZ searchPoint;

    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // neighbors within voxel search
    std::vector<int> pointIdxVec;
    if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z << ")"
                  << std::endl;

        for (size_t i = 0; i < pointIdxVec.size(); ++i)
        {
            std::cout << " " << cloud->points[pointIdxVec[i]].x
                      << " " << cloud->points[pointIdxVec[i]].y
                      << " " << cloud->points[pointIdxVec[i]].z << std::endl;
        }
    }

    // K nerest neighbor search
    int K = 10;

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquareDistance;

    std::cout << "K nearest neighbors search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z << ") with K=" << K << std::endl;

    if (octree.nearestKSearch(searchPoint, K, pointIdxVec, pointNKNSquareDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        {
            std::cout << " " << cloud->points[pointIdxNKNSearch[i]].x
                      << " " << cloud->points[pointIdxNKNSearch[i]].y
                      << " " << cloud->points[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquareDistance[i] << ")" << std::endl;
        }
    }

    // neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquareDistance;

    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            std::cout << " " << cloud->points[pointIdxRadiusSearch[i]].x
                      << " " << cloud->points[pointIdxRadiusSearch[i]].y
                      << " " << cloud->points[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquareDistance[i] << ")" << std::endl;
        }
    }

    return 0;
}