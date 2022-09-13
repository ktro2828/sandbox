#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

int user_data;

void viewerOnOff(pcl::visualization::PCLVisualizer& viewer) {
    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer) {
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    user_data++;
}

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // .pcd path is relative path from build/
    pcl::io::loadPCDFile("../../pcd/kitchen/Rf11.pcd", *cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    // use the following functions to get access to the underlying more advanced/poweful
    // PCLVisualizer

    // This will only get called once
    viewer.runOnVisualizationThreadOnce(viewerOnOff);

    // This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped()) {
        // you can also do cool processing here
        user_data++;
    }
    return 0;
}