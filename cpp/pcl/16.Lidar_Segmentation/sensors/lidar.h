#ifndef LIDAR_H
#define LIDAR_H

#include "../render/render.h"
#include <ctime>
#include <chrono>

const double pi = 3.1415;


struct Ray
{
  Vec3 origin;
  double resolution;
  Vec3 direction;
  Vec3 castPosition;
  double castDistance;


  Ray(Vec3)
}
#endif // LIDAR_H
