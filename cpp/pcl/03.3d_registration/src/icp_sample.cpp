// ICP algorithm with ROS
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>


class ICP_tf
{
public:
  ICP_tf();

private:
  // catch pointcloud2
  void cb_catch_cloud2_first(const sensor_msgs::PointCloud2::ConstPtr& cloud2);
  void cb_catch_cloud2_second(const sensor_msgs::PointCloud2::ConstPtr& cloud2);

  void transform(const ros::TimeEvent&);

  // create node handler
  ros::NodeHandle nh;

  ros::Publisher pub_pcl_first;

  ros::Subscriber sub_pcl_first;
  ros::Subscriber sub_pcl_second;

  // pointcloud msg
  sensor_msgs::PointCloud2 cloud2_first;
  sensor_msgs::PointCloud2 cloud2_second;

  // pcl_point_cloud
  pcl::PointCloud<pcl::PointXYZ> pcl_first;
  pcl::PointCloud<pcl::PointXYZ> pcl_second;

  // timer
  ros::Timer timer;
};


// constractor
ICP_tf::ICP_tf()
{
  pub_pcl_first = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/pcl_data", 10);

  sub_cloud2_first = nh.subscribe<sensor_msgs::PointCloud2> ("/cloud2_control_point_first", 100, &ICP_tf::cb_catch_cloud2_first, this);
  sub_cloud2_second = nh.subscribe<sensor_msgs::PointCloud2>("/cloud2_control_point_second", 100, &ICP_tf::cb_catch_cloud_second, this);

  timer = nh.createTimer(ros::Duration(1.0), &ICP_tf::transform, this);
}

//pointcloud cb
void ICP_tf::cb_catch_cloud2_first(const sensor_msgs::PointCloud2::ConstPtr& cloud2)
{
  this->cloud2_first = *cloud;
  pcl::fromROSMsg(this->cloud2_first, this->pcl_first);

  // ROS_INFO("first");
  // pub_pcl_first.publish(this->pcl_first);
}

void ICP_tf::cb_catch_cloud2_second(const sensor_msgs::PointCloud2::ConstPtr& cloud2)
{
  this->cloud2_second = *cloud2;
  pcl::fromROSMsg(this->cloud2_second, this->pcl_second);
}


// display function
void print4x4Matrix(const Eigen::Matrix4d& matrix)
{
  std::cout << "Homogeneous transformation matrix :" << std::endl;
  std::cout << matrix << std::endl;
}

// transform function
void ICP_tf::transform(const ros::TimerEvent&)
{
  // cloud_first
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  icp.setInputSource(this->pcl_first.makeShared());
  icp.seqInputTarget(this->pcl_second.makeShared());

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  // display transformation matrix
  Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identiry();
  tf_matrix = icp.getFinalTransformation().cast<double>();
  print4x4Matrix(tf_matrix);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp_transform");

  ICP_tf icp_tf;

  ros::spin();

  return 0;
}
