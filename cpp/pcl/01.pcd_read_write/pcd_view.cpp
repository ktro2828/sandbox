#include <pcl/visualization/pcl_visualizer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> threeCloudsVis (
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud3)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  // Red cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, single_color1, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");

  // Green cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");

  // Blue cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud3, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud3, single_color3, "cloud3");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud3");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr fillInCloudRandomData (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  cloud->width    = 5;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  return cloud;
}

int main (int argc, char** argv)
{
  // ランダムなデータを入れた点群を3つ用意
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  cloud1 = fillInCloudRandomData(cloud1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  cloud2 = fillInCloudRandomData(cloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
  cloud3 = fillInCloudRandomData(cloud3);


  // 点群の可視化
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = threeCloudsVis(cloud1, cloud2, cloud3);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}