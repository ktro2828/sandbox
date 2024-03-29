#ifndef _SIMPLE_OPENNI_VIEWER_H_
#define _SIMPLE_OPENNI_VIEWER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>


class SimpleOpenNIViewer 
{
    private:
        pcl::visualization::PCLVisualizer viewer;

    public:
        SimpleOpenNIViewer(): {
            
        }

        void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
        void run();
};

#endif //_SIMPLE_OPENNI_VIEWER_H_