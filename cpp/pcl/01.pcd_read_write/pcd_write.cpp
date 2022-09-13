#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
        
        /**
         * struct PointXYZ
         * {
         *    float x;
         *    float y;
         *    float z;
         * };
         */
    
    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    int status = pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    if (status == -1) {
        std::cerr << "Unsaved" << cloud.points.size() << "data points to test_pcd.pcd" << std::endl;
    } else {
        std::cout << "Saved" << cloud.points.size() << "data points to test_pcd.pcd" << std::endl;
    }

    for (size_t i = 0; i < cloud.points.size(); ++i){
        std::cout << "  " << cloud.points[i].x
                  << "  " << cloud.points[i].y
                  << "  " << cloud.points[i].z << std::endl;
    }

    return 0;
}