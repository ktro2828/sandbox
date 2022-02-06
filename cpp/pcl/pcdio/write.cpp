#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // std::cout << "Enter width:" << std::endl;
  // std::cin >> cloud.width;
  // std::cout << "Enter height:" << std::endl;
  // std::cin >> cloud.height;

  cloud.width = 5;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i=0; cloud.points.size(); ++i)
    {
      cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
      cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
      cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

  std::string savepth;
  if (argc > 2)
    savepth = argv[1];
  else
    savepth = "test.pcd";
  // int savePCDFileASCII(const std::string& filename,
  //                      const pcl::PointCloud<PointT>& cloud)
  int status = pcl::io::savePCDFileASCII(savepth, cloud);
  if (status == -1)
    {
      std::cerr << "Failed to save "
		<< cloud.points.size()
		<< " data points to "
		<< savepth
		<< std::endl;      
    }
  else
    {
      std::cerr << "Success to save "
		<< cloud.points.size()
		<< " data points to "
		<< savepth
		<<std::endl;
    }
}
