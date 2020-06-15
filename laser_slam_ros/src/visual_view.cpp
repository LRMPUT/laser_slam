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

    // TODO move to params or detect
    int horRes = 1024;
    int vertRes = 64;
    float vertRange = 45.0 * M_PI / 180.0;
    float rangeThresh = 1.0;

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
          float horRange = std::sqrt(x*x + y*y);

          float horAngle = atan2(-y, x);
          if (horAngle < 0) {
            horAngle += 2.0 * M_PI;
          }
          float vertAngle = atan2(z, horRange);

          int horCoord = int(horAngle / (2 * M_PI) * horRes);
          int vertCoord = int((vertRange / 2.0 - vertAngle) / vertRange * vertRes);

          if(horCoord < 0 || horRes <= horCoord || vertCoord < 0 || vertRes <= vertCoord) {
            cout << i << ": (" << x << ", " << y << ", " << z << ")" << endl;
            cout << "horAngle = " << horAngle << "\tvertAngle = " << vertAngle << "\tintensity = " << intVal << endl;
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

VisualView::MatrixInt
VisualView::getMask(const laser_slam_ros::PointCloud &point_cloud) {
    return laser_slam_ros::VisualView::MatrixInt();
}

}