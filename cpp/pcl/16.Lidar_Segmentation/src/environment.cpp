// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

#include "sensor/lidar.h"
#include "render/render.h"
#iclude "processPointClouds.h"
#include "processpointclouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  Car egoCar(Vect3(0,  0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(  Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(  Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(  Vect3(-12,4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (RenderScene) {
    renderHigheway(ciewer);
    
  }
}
