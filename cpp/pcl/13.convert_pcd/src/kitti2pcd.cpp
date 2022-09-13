#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/features/normal_3d_omp/h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>

namespace po = boost::program_options;


int main(int argc, char** argv)
{
  // The file to read from
  std::string infile;

  // The file to output
  std::string outfile;

  // Declare the supported options
  po::options_description desc("Program options");
  desc.add_options()
    // Options
    ("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
    ("outfile", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud & normal to")
    ;
  // Parse the conmmand line
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // Print help
  if (vm.count("help"))
    {
      std::cout << desc << std::endl;
      return false;
    }

  // Process options
  po::notify(vm);

  // load point cloud
  fstream input(infile.c_str(), ios::in | ios::binary);
  if (!input.good())
    {
      std::cerr << "Could not read" << infile << std::endl;
      exit(EXIT_FAILUARE);
    }
  input.seekg(0, ios::beg);

  pcl::PointCloud<PointXYZI>::Ptr points(new pcl::PointCloud<PointXYZI>);

  int i;
  for (i = 0; input.good() && !input.eof(); i++)
    {
      PointXYZI point;
      input.read((char *) &point.x, 3 * sizeof(float));
      input.read((char *) &point.intensity, sizeof(float));
      points->push_back(point);
    }
  input.close();

  std::cout << "Read KITTI point cloud with " << i << " points, writing to " << outfile << std::endl;

  pcl::PCDWriter writer;

  // Save DoN features
  writer.write<PointXYZ> (outfile, *points, false);

  return 0;
}
