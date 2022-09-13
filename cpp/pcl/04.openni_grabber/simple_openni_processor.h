#ifndef _SIMPLE_OPENNI_PROCESSOR_H_
#define _SIMPLE_OPENNI_PROCESSOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>


class SimpleOpenNIProcessor
{
    public:
        void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
        void run();
};

#endif // _SIMPLE_OPENNI_PROCESSOR_H_