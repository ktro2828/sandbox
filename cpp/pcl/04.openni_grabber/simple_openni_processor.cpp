#include "simple_openni_processor.h"

void SimpleOpenNIProcessor::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{

    static unsigned count = 0;
    static double last = pcl::getTime();

    if (++count == 30)
    {
        double now = pcl::getTime();
        std::cout << "distance of center pixel :" << cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z
                  << " mm. Average framerate: " << double(count) / double(now) << " Hz" << std::endl;

        count = 0;
        last = now;
    }
}


void SimpleOpenNIProcessor::run()
{
    // create a new grabber for OpenNI devices
    pcl::Grabber *interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f =
        boost::bind(&SimpleOpenNIProcessor::cloud_cb_, this, 1);

    // conect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback(f);

    // start receiving point clouds
    interface->start();

    // wait until user quits progtram with Ctrl-C, but no busy-waiting->sleep(1);
    while (true) {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    }

    // stop the grabber
    interface->stop();
}