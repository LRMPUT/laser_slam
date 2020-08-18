//
// Created by janw on 02.06.2020.
//

// STL
#include <iostream>
#include <algorithm>
#include <sstream>
#include <string>

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>


#include "laser_slam_ros/visual_view.hpp"


using std::cout;
using std::endl;

namespace laser_slam_ros {

VisualView::VisualView()
  : isCompressed(false)
{
  // ++ids;
  // LOG(INFO) << "allocated " << ids;
}

VisualView::VisualView(const laser_slam::LaserScan &iscan, const laser_slam::Pose &ipose)
  : isCompressed(false),
    pixelOffsets{0, 6, 12, 18},
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
    dirs.resize(vertRes * horRes, Eigen::Vector3f::Zero());
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
          int horCoord = (i - pixelOffsets[vertCoord % 4] + horRes) % horRes;

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
          dirs[vertCoord * horRes + horCoord] = Eigen::Vector3f(x, y, z).normalized();
        }
    }

    ++ids;
    // LOG(INFO) << "allocated " << ids;
}

// VisualView::VisualView(VisualView &&other) :
//     pixelOffsets(std::move(other.pixelOffsets)),
//     vertAngles(std::move(other.vertAngles)),
//     pose(std::move(other.pose)),
//     time_ns(std::move(other.time_ns)),
//     intensity(std::move(other.intensity)),
//     range(std::move(other.range)),
//     count(std::move(other.count)),
//     dirs(std::move(other.dirs)) {
//   ++ids;
// }

// VisualView::VisualView(const VisualView &other) :
//     pixelOffsets(other.pixelOffsets),
//     vertAngles(other.vertAngles),
//     pose(other.pose),
//     time_ns(other.time_ns),
//     intensity(other.intensity),
//     range(other.range),
//     count(other.count),
//     dirs(other.dirs) {
//   ++ids;
//   // LOG(INFO) << "allocated " << ids;
// }

// VisualView::~VisualView(){
//   --ids;
//   // LOG(INFO) << "allocated " << ids;
// }

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

std::pair<int, int> VisualView::getClosestDir(const float &x, const float &y, const float &z,
                                              const int &r1,
                                              const int &c1,
                                              const int &r2,
                                              const int &c2) const {
  Eigen::Vector3f dir(x, y, z);
  dir.normalize();
  std::pair<int, int> bestCoord(-1, -1);
  float bestDiff = std::numeric_limits<float>::max();
  float bestDiffAll = std::numeric_limits<float>::max();
  for(int r = r1; r < r2; ++r){
    for(int c = c1; c < c2; ++c){
      if(range(r, c) > rangeThresh) {
        // 1 - cosine(alpha)
        float diff = 1.0f - dir.dot(dirs[r * horRes + c]);
        if ((diff < 1.0f - cos(dirThresh * M_PI / 180.0)) && diff < bestDiff) {
          bestCoord.first = r;
          bestCoord.second = c;
          bestDiff = diff;
        }
        if (diff < bestDiffAll) {
          bestDiffAll = diff;
        }
      }
    }
  }
  // if(bestCoord.first == -1 && bestCoord.second == -1){
  //   cout << "bestDiff = " << acos(1.0 - bestDiffAll) * 180.0 / M_PI << endl;
  // }

  return bestCoord;
}

VisualView::MatrixInt
VisualView::getMask(const laser_slam_ros::PointCloud &point_cloud) const {
  MatrixInt mask;
  mask.resize(vertRes, horRes);
  mask.setZero();

  for(int p = 0; p < point_cloud.size(); ++p) {
    Eigen::Vector3d ptSensor = pose.T_w.inverseTransform(point_cloud.at(p).getVector3fMap().cast<double>());

    // narrowing down search area
    int horCoordComp = getHorCoord(ptSensor(0), ptSensor(1), ptSensor(2));
    int vertCoordComp = getVertCoord(ptSensor(0), ptSensor(1), ptSensor(2));
    int r1 = std::max(0, vertCoordComp - 32);
    int r2 = std::min(vertRes - 1, vertCoordComp + 32);
    int c1 = std::max(0, horCoordComp - 32);
    int c2 = std::min(horRes - 1, horCoordComp + 32);

    std::pair<int, int> coord = getClosestDir(ptSensor(0), ptSensor(1), ptSensor(2),
                                              r1, c1, r2, c2);
    int horCoord = coord.second;
    int vertCoord = coord.first;


    // cout << p << ": (" << ptSensor(0) << ", " << ptSensor(1) << ", " << ptSensor(2) << ")" << endl;
    // cout << "horCoord1 = " << horCoord << "\tvertCoord1 = " << vertCoord << endl;
    // cout << "horCoord2 = " << horCoordComp << "\tvertCoord2 = " << vertCoordComp << endl;

    if(0 <= horCoord && horCoord < horRes && 0 <= vertCoord && vertCoord < vertRes) {
      if(ptSensor.norm() < range(vertCoord, horCoord) + occlusionThresh) {
        mask(vertCoord, horCoord) += 1;
      }
    }
    // else{
    //   cout << "horCoord1 = " << horCoord << "\tvertCoord1 = " << vertCoord << endl;
    //   cout << "horCoord2 = " << horCoordComp << "\tvertCoord2 = " << vertCoordComp << endl;
    // }
  }
  return mask;
}

std::vector<uint8_t> VisualView::compressData(const std::vector<uint8_t> &data) {
  std::stringstream compressed;
  std::stringstream decompressed;
  decompressed << std::string((char*)data.data(), data.size());
  boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
  out.push(boost::iostreams::zlib_compressor());
  out.push(decompressed);
  boost::iostreams::copy(out, compressed);
  std::string compressedStr = compressed.str();
  return std::vector<uint8_t>(compressedStr.begin(), compressedStr.end());
}

std::vector<uint8_t> VisualView::decompressData(const std::vector<uint8_t> &dataComp) {
  std::stringstream compressed;
  std::stringstream decompressed;
  compressed << std::string((char*)dataComp.data(), dataComp.size());
  boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
  in.push(boost::iostreams::zlib_decompressor());
  in.push(compressed);
  boost::iostreams::copy(in, decompressed);
  std::string decompressedStr = decompressed.str();
  return std::vector<uint8_t>(decompressedStr.begin(), decompressedStr.end());
}

void VisualView::compress() {
  if(intensity.size() > 0){
    std::vector<uint8_t> data((uint8_t*)intensity.data(), (uint8_t*)intensity.data() + intensity.size() * sizeof(float));
    intensityComp = compressData(data);
    intensity.resize(0, 0);
    // cout << "data.size() = " << data.size() << endl;
    // std::vector<uint8_t> dataComp = compressData(data);
    // cout << "dataComp.size() = " << dataComp.size() << endl;
    // std::vector<uint8_t> data2 = decompressData(dataComp);
    // cout << "data2.size() = " << data2.size() << endl;
    // Matrix intensity2 = Eigen::Map<Matrix>((float*)data2.data(), vertRes, horRes);
    // if(intensity != intensity2) {
    //   cout << endl << "Matrices dont match" << endl << endl;
    //   cout << intensity << endl;
    //   cout << intensity2 << endl;
    // }
    // else{
    //   cout << endl << "Matrices match :)" << endl << endl;
    // }
  }
  if(range.size() > 0){
    std::vector<uint8_t> data((uint8_t*)range.data(), (uint8_t*)range.data() + range.size() * sizeof(float));
    rangeComp = compressData(data);
    range.resize(0, 0);
  }
  if(count.size() > 0){
    std::vector<uint8_t> data((uint8_t*)count.data(), (uint8_t*)count.data() + count.size() * sizeof(int));
    countComp = compressData(data);
    count.resize(0, 0);
  }
  if(dirs.size() > 0){
    std::vector<uint8_t> data((uint8_t*)dirs.data(), (uint8_t*)dirs.data() + dirs.size() * sizeof(Eigen::Vector3f));
    dirsComp = compressData(data);
    dirs.clear();
  }
  isCompressed = true;
}

void VisualView::decompress() {
  if(intensityComp.size() > 0) {
    std::vector<uint8_t> data = decompressData(intensityComp);
    intensity = Eigen::Map<Matrix>((float*)data.data(), vertRes, horRes);
    intensityComp.clear();
    // cout << "intensity decompressed" << endl;
  }
  if(rangeComp.size() > 0) {
    std::vector<uint8_t> data = decompressData(rangeComp);
    range = Eigen::Map<Matrix>((float*)data.data(), vertRes, horRes);
    rangeComp.clear();
    // cout << "range decompressed" << endl;
  }
  if(countComp.size() > 0) {
    std::vector<uint8_t> data = decompressData(countComp);
    count = Eigen::Map<MatrixInt>((int*)data.data(), vertRes, horRes);
    countComp.clear();
    // cout << "count decompressed" << endl;
  }
  if(dirsComp.size() > 0) {
    std::vector<uint8_t> data = decompressData(dirsComp);
    dirs = std::vector<Eigen::Vector3f>((Eigen::Vector3f*)data.data(), (Eigen::Vector3f*)data.data() + vertRes * horRes);
    dirsComp.clear();
    // cout << "dirs decompressed" << endl;
  }
  isCompressed = false;
}


int VisualView::ids = 0;

}