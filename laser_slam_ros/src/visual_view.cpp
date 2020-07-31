//
// Created by janw on 02.06.2020.
//

// STL
#include <iostream>
#include <algorithm>

#include "laser_slam_ros/visual_view.hpp"


using std::cout;
using std::endl;

namespace laser_slam_ros {

VisualView::VisualView() {

}

VisualView::VisualView(const laser_slam::LaserScan &iscan, const laser_slam::Pose &ipose)
  : pixelOffsets{0, 6, 12, 18},
    vertAngles{-15.654, -15.062, -14.497, -13.947, -13.489, -12.908, -12.337, -11.803,
              -11.347, -10.78, -10.224, -9.675, -9.239, -8.664, -8.118, -7.574,
              -7.131, -6.575, -6.019, -5.47, -5.037, -4.478, -3.921, -3.382,
              -2.938, -2.274, -1.84, -1.282, -0.842, -0.29, 0.262, 0.805,
              1.246, 1.799, 2.346, 2.894, 3.338, 3.891, 4.442, 4.989,
              5.443, 5.977, 6.535, 7.079, 7.54, 8.086, 8.63, 9.189,
              9.633, 10.182, 10.734, 11.309, 11.751, 12.305, 12.852, 13.442,
              13.91, 14.444, 15.012, 15.615, 16.073, 16.629, 17.217, 17.84}
{

    pose = ipose;
    time_ns = iscan.time_ns;

    intensity.resize(vertRes, horRes);
    intensity.setZero();
    range.resize(vertRes, horRes);
    range.setZero();
    count.resize(vertRes, horRes);
    count.setZero();
    unsigned intDim = 0;
    if(iscan.scan.descriptorExists("intensity")){
        intDim = iscan.scan.getDescriptorDimension("intensity");
        // cout << "intDim = " << intDim << endl;
    }
    else{
        LOG(ERROR) << "No intensity data in laser scan";
    }
    // cout << "iscan.scan.getNbPoints() = " << iscan.scan.getNbPoints() << endl;
    for (size_t i = 0u; i < iscan.scan.getNbPoints(); ++i) {
        float x = iscan.scan.features(0,i);
        float y = iscan.scan.features(1,i);
        float z = iscan.scan.features(2,i);

        float intVal = iscan.scan.descriptors(intDim, i);
        float rangeVal = std::sqrt(x*x + y*y + z*z);

        if(rangeVal > rangeThresh) {
          int vertCoord = i / horRes;
          int horCoord = (i - pixelOffsets[vertCoord % 4]) % horRes;

          // int horCoord = getHorCoord(x, y, z);
          // int vertCoord = getVertCoord(x, y, z);
          // if(horCoord < 0 || horRes <= horCoord || vertCoord < 0 || vertRes <= vertCoord) {
          //   cout << i << ": (" << x << ", " << y << ", " << z << ")" << endl;
          //   // cout << "horAngle = " << horAngle << "\tvertAngle = " << vertAngle << "\tintensity = " << intVal << endl;
          //   cout << "horCoord = " << horCoord << "\tvertCoord = " << vertCoord << endl;
          //
          //   LOG(ERROR) << "Wrong image coordinates";
          //   throw "Wrong image coordinates";
          // }

          intensity(vertCoord, horCoord) = intVal;
          range(vertCoord, horCoord) = rangeVal;
          count(vertCoord, horCoord) += 1;
          dirs[vertCoord][horCoord] = Eigen::Vector3f(x, y, z).normalized();
        }
    }
}

float VisualView::getHorAngle(const float &x, const float &y, const float &z) const {
  float horAngle = atan2(-y, x);
  if (horAngle < 0) {
    horAngle += 2.0 * M_PI;
  }

  return horAngle;
}

float VisualView::getVertAngle(const float &x, const float &y, const float &z) const {
  float horRange = std::sqrt(x*x + y*y);
  float vertAngle = atan2(z, horRange);

  return vertAngle;
}

int VisualView::getHorCoord(const float &x, const float &y, const float &z) const {
  float horAngle = getHorAngle(x, y, z);

  int horCoord = int(horAngle / (2 * M_PI) * horRes);

  return horCoord;
}

int VisualView::getVertCoord(const float &x, const float &y, const float &z) const {
  float vertAngle = getVertAngle(x, y, z);

  auto it = std::lower_bound(vertAngles.begin(), vertAngles.end(), vertAngle * 180.0 / M_PI);
  if (it != vertAngles.begin()){
    if(std::abs(*(it-1) - vertAngle * 180.0 / M_PI) < std::abs(*it - vertAngle * 180.0 / M_PI)){
      --it;
    }
  }
  // int vertCoord = int((vertRange / 2.0 - vertAngle) / vertRange * (vertRes - 1));
  int vertCoord = vertAngles.end() - it - 1;

  return vertCoord;
}

std::pair<int, int> VisualView::getClosestDir(const float &x, const float &y, const float &z) const {
  Eigen::Vector3f dir(x, y, z);
  dir.normalize();
  std::pair<int, int> bestCoord(-1, -1);
  float bestDiff = std::numeric_limits<float>::max();
  for(int r = 0; r < vertRes; ++r){
    for(int c = 0; c < horRes; ++c){
      if(range(r, c) > rangeThresh) {
        // 1 - cosine(alpha)
        float diff = 1.0f - dir.dot(dirs[r][c]);
        if (diff < bestDiff) {
          bestCoord.first = r;
          bestCoord.second = c;
          bestDiff = diff;
        }
      }
    }
  }

  return bestCoord;
}

VisualView::MatrixInt
VisualView::getMask(const laser_slam_ros::PointCloud &point_cloud) const {
  MatrixInt mask;
  mask.resize(vertRes, horRes);
  mask.setZero();

  for(int p = 0; p < point_cloud.size(); ++p) {
    Eigen::Vector3d ptSensor = pose.T_w.inverseTransform(point_cloud.at(p).getVector3fMap().cast<double>());

    std::pair<int, int> coord = getClosestDir(ptSensor(0), ptSensor(1), ptSensor(2));
    int horCoord = coord.second;
    int vertCoord = coord.first;

    // int horCoordComp = getHorCoord(ptSensor(0), ptSensor(1), ptSensor(2));
    // int vertCoordComp = getVertCoord(ptSensor(0), ptSensor(1), ptSensor(2));
    // cout << p << ": (" << ptSensor(0) << ", " << ptSensor(1) << ", " << ptSensor(2) << ")" << endl;
    // cout << "horCoord1 = " << horCoord << "\tvertCoord1 = " << vertCoord << endl;
    // cout << "horCoord2 = " << horCoordComp << "\tvertCoord2 = " << vertCoordComp << endl;

    if(0 <= horCoord && horCoord < horRes && 0 <= vertCoord && vertCoord < vertRes) {
      if(ptSensor.norm() < range(vertCoord, horCoord) + occlusionThresh) {
        mask(vertCoord, horCoord) += 1;
      }
    }
  }
  return mask;
}

}