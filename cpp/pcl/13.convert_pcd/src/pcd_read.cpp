#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>


int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
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

    // FIXME: possible race condition here:
    user_data++;
}

int main(int argc, char ** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (argc < 2)
      {
	std::cout << "Usage: ./pcd_read pcd_file.pcd" << std::endl;
	return -1;
      }

    std::string filename = argv[1];

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return -1;
    }

    std::cout << "Loaded\n"
              << cloud->width * cloud->height
              << "data points from test_pcd.pcd with the following fields:"
              << std::endl;

    // for (size_t i = 0; i < cloud->points.size(); ++i) {
    //     std::cout << "  " << cloud->points[i].x
    //               << "  " << cloud->points[i].y
    //               << "  " << cloud->points[i].z << std::endl;
    // }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    // use the following functions to get access to the underlyoing more advanced/powerful
    // PCLVisualizer

    // This will only get called once
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    // This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewerPsycho);

    while (!viewer.wasStopped()) 
    {
        user_data++;
    }

    return 0;
}
