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
  o.y = 0.0;
  o.z = 0.0;
  // bool addSphere(const PointT& ceter,
  //                double radius,
  //                const std::string& id="sphere",
  //                int viewport=0)
  viewer.addSphere(o, 0.25, "sphere", 0);
  std::cout << "i only run once" << std::endl;
}


void viewerPsycho(pcl::visualization::PCLVisualizer& viewer) {
  static unsigned count = 0;
  std::stringstream ss;
  ss << "Once per viewer loop: " << count++;
  // bool removeShape(const std::string& id="cloud",
  //                  int viewport=0)
  viewer.removeShape("text", 0);
  viewer.addText(ss.str(), 200, 300, "text", 0);

  // @todo: possible race condition here
  user_data++;
}

int main(int argc, char** argv) {
  // Buffer for loaded point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (argc < 2)
    {
      std::cerr << "Usage: ./read_pcd PCDFILE.pcd" << std::endl;
      return -1;
    }

  std::string filename = argv[1];

  // int loadPCDFILE<PointT>(const std::string& filename
  //                         pcl::PCLPointCloud2 &cloud)
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
    {
      std::cerr << "Cannot read " << filename << std::endl;
      return -1;
    }

  std::cout << "Loaded\n"
	    << cloud->width * cloud->height
	    << " data points from "
	    << filename
	    << std::endl;

  pcl::visualization::CloudViewer viewer("Cloud Viewer");

  // void showCloud(const GrayCloud::ConstPtr& cloud,
  //                const std::string& cloudname="cloud")
  viewer.showCloud(cloud);

  viewer.runOnVisualizationThreadOnce(viewerOneOff);

  viewer.runOnVisualizationThread(viewerPsycho);

  while (!viewer.wasStopped())
    {
      user_data++;
    }

  return 0;
}

