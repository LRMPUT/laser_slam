//
// Created by janw on 02.06.2020.
//

// STL
#include <iostream>

#include "laser_slam_ros/visual_view.hpp"


using std::cout;
using std::endl;

namespace laser_slam_ros {

VisualView::VisualView() {

}

VisualView::VisualView(const laser_slam::LaserScan &iscan, const laser_slam::Pose &ipose) {
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
    for (size_t i = 0u; i < iscan.scan.getNbPoints(); ++i) {
        float x = iscan.scan.features(0,i);
        float y = iscan.scan.features(1,i);
        float z = iscan.scan.features(2,i);

        float intVal = iscan.scan.descriptors(intDim, i);
        float rangeVal = std::sqrt(x*x + y*y + z*z);

        if(rangeVal > rangeThresh) {
          int horCoord = getHorCoord(x, y, z);
          int vertCoord = getVertCoord(x, y, z);

          if(horCoord < 0 || horRes <= horCoord || vertCoord < 0 || vertRes <= vertCoord) {
            cout << i << ": (" << x << ", " << y << ", " << z << ")" << endl;
            // cout << "horAngle = " << horAngle << "\tvertAngle = " << vertAngle << "\tintensity = " << intVal << endl;
            cout << "horCoord = " << horCoord << "\tvertCoord = " << vertCoord << endl;

            LOG(ERROR) << "Wrong image coordinates";
            throw "Wrong image coordinates";
          }

          intensity(vertCoord, horCoord) = intVal;
          range(vertCoord, horCoord) = rangeVal;
          count(vertCoord, horCoord) += 1;
        }
    }
}

int VisualView::getHorCoord(const float &x, const float &y, const float &z) const {
  float horAngle = atan2(-y, x);
  if (horAngle < 0) {
    horAngle += 2.0 * M_PI;
  }

  int horCoord = int(horAngle / (2 * M_PI) * horRes);

  return horCoord;
}

int VisualView::getVertCoord(const float &x, const float &y, const float &z) const {
  float horRange = std::sqrt(x*x + y*y);

  float vertAngle = atan2(z, horRange);
  int vertCoord = int((vertRange / 2.0 - vertAngle) / vertRange * vertRes);

  return vertCoord;
}

VisualView::MatrixInt
VisualView::getMask(const laser_slam_ros::PointCloud &point_cloud) const {
  MatrixInt mask;
  mask.resize(vertRes, horRes);
  mask.setZero();

  for(int p = 0; p < point_cloud.size(); ++p) {
    Eigen::Vector3d ptSensor = pose.T_w.inverseTransform(point_cloud.at(p).getVector3fMap().cast<double>());

    int horCoord = getHorCoord(ptSensor(0), ptSensor(1), ptSensor(2));
    int vertCoord = getVertCoord(ptSensor(0), ptSensor(1), ptSensor(2));

    // cout << p << ": (" << ptSensor(0) << ", " << ptSensor(1) << ", " << ptSensor(2) << ")" << endl;
    // cout << "horCoord = " << horCoord << "\tvertCoord = " << vertCoord << endl;

    if(0 <= horCoord && horCoord < horRes && 0 <= vertCoord && vertCoord < vertRes) {
      mask(vertCoord, horCoord) += 1;
    }
  }
  return mask;
}

}