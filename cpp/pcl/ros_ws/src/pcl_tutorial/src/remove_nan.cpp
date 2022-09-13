#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>


void callback(sensor_msgs::PointCloud2 pc2) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // convert sensor_msgs::PointCloud2 => pcll::PointXYZ
    pcl::fromROSMsg(pc2, *cloud_nan);

    // remove Nan value
    std::vector<int> nan_index;
    pcl::removeNaNFromPointCloud(*cloud_nan, *cloud, nan_index);

    // create KD-tree(makes seach faster)
    pcl::kdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::kdTreeFLANN<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // set dst valiables
    double radious = 0.1;
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    unsigned int max_nn = 1;

    // target point's position
    pcl::PointXYZ p;
    p.x = 0.5;
    p.y = 0.5;
    p.z = 0.5;

    if (k_indices.size() == 0) {
        return;
    }

    pcl::PointXYZ result = cloud->points[k_indices[0]];

    ROS_INFO("A nearest point of (0.5, 0.5, 0.5) is ...\nx: %lf, y:%lf, z:%lf", result.x, result.y, result.z);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "z_at_xy");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, callback);

    ros::spin();
    return 0;
}