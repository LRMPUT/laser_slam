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

#define PRINT(x) std::cout << #x << " = " << x << std::endl

using std::cout;
using std::endl;

#include <signal.h>
#include <termios.h>
#include <stdio.h>
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

namespace laser_slam_ros {

VisualView::VisualView()
  : isCompressedFlag(false)
{
  // ++ids;
  // LOG(INFO) << "allocated " << ids;
}

VisualView::VisualView(const laser_slam::LaserScan &iscan,
                       const laser_slam::Pose &ipose,
                       const int &ihorRes,
                       const int &ivertRes,
                       bool iorganized)
  : isCompressedFlag(false),
    horRes(ihorRes),
    vertRes(ivertRes),
    organized(iorganized),
    pixelOffsets{0, 6, 12, 18},
    // vertAngles{-15.654, -15.062, -14.497, -13.947, -13.489, -12.908, -12.337, -11.803,
    //           -11.347, -10.78, -10.224, -9.675, -9.239, -8.664, -8.118, -7.574,
    //           -7.131, -6.575, -6.019, -5.47, -5.037, -4.478, -3.921, -3.382,
    //           -2.938, -2.274, -1.84, -1.282, -0.842, -0.29, 0.262, 0.805,
    //           1.246, 1.799, 2.346, 2.894, 3.338, 3.891, 4.442, 4.989,
    //           5.443, 5.977, 6.535, 7.079, 7.54, 8.086, 8.63, 9.189,
    //           9.633, 10.182, 10.734, 11.309, 11.751, 12.305, 12.852, 13.442,
    //           13.91, 14.444, 15.012, 15.615, 16.073, 16.629, 17.217, 17.84}
    vertAngles{-24.8       , -24.37460317, -23.94920635, -23.52380952,
               -23.0984127 , -22.67301587, -22.24761905, -21.82222222,
               -21.3968254 , -20.97142857, -20.54603175, -20.12063492,
               -19.6952381 , -19.26984127, -18.84444444, -18.41904762,
               -17.99365079, -17.56825397, -17.14285714, -16.71746032,
               -16.29206349, -15.86666667, -15.44126984, -15.01587302,
               -14.59047619, -14.16507937, -13.73968254, -13.31428571,
               -12.88888889, -12.46349206, -12.03809524, -11.61269841,
               -11.18730159, -10.76190476, -10.33650794,  -9.91111111,
               -9.48571429,  -9.06031746,  -8.63492063,  -8.20952381,
               -7.78412698,  -7.35873016,  -6.93333333,  -6.50793651,
               -6.08253968,  -5.65714286,  -5.23174603,  -4.80634921,
               -4.38095238,  -3.95555556,  -3.53015873,  -3.1047619 ,
               -2.67936508,  -2.25396825,  -1.82857143,  -1.4031746 ,
               -0.97777778,  -0.55238095,  -0.12698413,   0.2984127 ,
               0.72380952,   1.14920635,   1.57460317,   2.        }
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
    if (organized) {
      for (size_t i = 0u; i < iscan.scan.getNbPoints(); ++i) {
        float x = iscan.scan.features(0, i);
        float y = iscan.scan.features(1, i);
        float z = iscan.scan.features(2, i);

        float intVal = iscan.scan.descriptors(intDim, i);
        float rangeVal = std::sqrt(x * x + y * y + z * z);

        if (rangeVal > rangeThresh) {
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
    }
    // do interpolation
    else {
      // std::cout << "interpolating intensity image" << std::endl;
      // std::cout << "binning" << std::endl;
      std::vector<std::vector<std::vector<int>>> bins(vertRes,
                                                     std::vector<std::vector<int>>(horRes, std::vector<int>()));
      for (size_t i = 0u; i < iscan.scan.getNbPoints(); ++i) {
        float x = iscan.scan.features(0, i);
        float y = iscan.scan.features(1, i);
        float z = iscan.scan.features(2, i);

        float intVal = iscan.scan.descriptors(intDim, i);
        float rangeVal = std::sqrt(x * x + y * y + z * z);

        if (rangeVal > rangeThresh) {
          int horCoord = getHorCoordLow(x, y, z);
          int vertCoord = getVertCoordLow(x, y, z);

          if(0 <= horCoord && horCoord < horRes && 0 <= vertCoord && vertCoord < vertRes) {
            // std::cout << "adding to bin (" << vertCoord << ", " << horCoord << ")" << std::endl;
            bins[vertCoord][horCoord].push_back(i);
          }
        }
      }

      // std::cout << "interpolating" << std::endl;
      for(int r = 0; r < vertRes; ++r) {
        for (int c = 0; c < horRes; ++c) {
          // PRINT(r);
          // PRINT(c);
          Eigen::Vector3f dir = getDir(r, c);
          float horAngle = getHorAngle(dir(0), dir(1), dir(2));
          float vertAngle = getVertAngle(dir(0), dir(1), dir(2));
          // PRINT(dir.transpose());
          // PRINT(horAngle);
          // PRINT(vertAngle);

          int nh00 = getClosest(iscan, bins[r][c], dir);
          int nh10 = -1;
          if (r > 0) {
            nh10 = getClosest(iscan, bins[r - 1][c], dir);
          }
          // there is always a neighboring column
          int nh01 = getClosest(iscan, bins[r][(c - 1 + horRes) % horRes], dir);
          int nh11 = -1;
          if (r > 0) {
            nh11 = getClosest(iscan, bins[r - 1][(c - 1 + horRes) % horRes], dir);
          }

          // PRINT(nh00);
          // PRINT(nh10);
          // PRINT(nh01);
          // PRINT(nh11);
          Eigen::Vector4f pt = Eigen::Vector4f::Zero();
          if (nh00 >= 0 && nh10 >= 0 && nh01 >= 0 && nh11 >= 0) {
            Eigen::Vector4f pt00 = getPoint(iscan, nh00, intDim);
            Eigen::Vector4f pt10 = getPoint(iscan, nh10, intDim);
            Eigen::Vector4f pt01 = getPoint(iscan, nh01, intDim);
            Eigen::Vector4f pt11 = getPoint(iscan, nh11, intDim);

            // first interpolate horizontally
            Eigen::Vector4f pt0 = interpolateHor(pt01, pt00, horAngle);
            Eigen::Vector4f pt1 = interpolateHor(pt11, pt10, horAngle);
            // then vertically
            pt = interpolateVert(pt0, pt1, vertAngle);
            // {
            //   PRINT(r);
            //   PRINT(c);
            //   PRINT(dir.transpose());
            //   PRINT(horAngle);
            //   PRINT(vertAngle);
            //   PRINT(getHorAngle(pt01(0), pt01(1), pt01(2)));
            //   PRINT(getHorAngle(pt00(0), pt00(1), pt00(2)));
            //   PRINT(nh00);
            //   PRINT(nh10);
            //   PRINT(nh01);
            //   PRINT(nh11);
            //   PRINT(pt00.transpose());
            //   PRINT(pt10.transpose());
            //   PRINT(pt01.transpose());
            //   PRINT(pt11.transpose());
            //   PRINT(pt0.transpose());
            //   PRINT(pt1.transpose());
            //   PRINT(pt.transpose());
            //   while (getch() != 'n') {
            //
            //   }
            // }
          }
          else if (nh00 >= 0 && nh10 >= 0) {
            Eigen::Vector4f pt00 = getPoint(iscan, nh00, intDim);
            Eigen::Vector4f pt10 = getPoint(iscan, nh10, intDim);

            // PRINT(pt00.transpose());
            // PRINT(pt10.transpose());
            pt = interpolateVert(pt00, pt10, vertAngle);
          }
          else if (nh01 >= 0 && nh11 >= 0) {
            Eigen::Vector4f pt01 = getPoint(iscan, nh01, intDim);
            Eigen::Vector4f pt11 = getPoint(iscan, nh11, intDim);

            // PRINT(pt01.transpose());
            // PRINT(pt11.transpose());
            // interpolate vertically
            pt = interpolateVert(pt01, pt11, vertAngle);
          }
          else if (nh00 >= 0 && nh01 >= 0) {
            Eigen::Vector4f pt00 = getPoint(iscan, nh00, intDim);
            Eigen::Vector4f pt01 = getPoint(iscan, nh01, intDim);

            // PRINT(pt00.transpose());
            // PRINT(pt01.transpose());
            // interpolate horizontally
            pt = interpolateHor(pt01, pt00, horAngle);
          }
          else if (nh10 >= 0 && nh11 >= 0) {
            Eigen::Vector4f pt10 = getPoint(iscan, nh10, intDim);
            Eigen::Vector4f pt11 = getPoint(iscan, nh11, intDim);

            // PRINT(pt10.transpose());
            // PRINT(pt11.transpose());
            // interpolate horizontally
            pt = interpolateHor(pt11, pt10, horAngle);
          }
          else {
            // chose the closest
            int nh = getClosest(iscan, std::vector<int>{nh00, nh10, nh01, nh11}, dir);
            if (nh >= 0) {
              pt = getPoint(iscan, nh, intDim);
            }
          }
          // PRINT(pt.transpose());
          if (pt != Eigen::Vector4f::Zero()) {
            intensity(r, c) = pt(3);
            range(r, c) = pt.head<3>().norm();
            dirs[r * horRes + c] = pt.head<3>().normalized();
            count(r, c) += 1;
          }
        }
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

float VisualView::angDiff(const float &a1, const float &a2) const {
  float d = a1 - a2;
  if (d < -M_PI) {
    d += 2 * M_PI;
  }
  else if(M_PI < d) {
    d -= 2 * M_PI;
  }
  return d;
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

  // round to nearest integer
  int horCoord = int(horAngle / (2 * M_PI) * horRes + 0.5);

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


int VisualView::getHorCoordLow(const float &x, const float &y, const float &z) const {
  float horAngle = getHorAngle(x, y, z);

  // round to nearest integer
  int horCoord = int(horAngle / (2 * M_PI) * horRes);

  return horCoord;
}

int VisualView::getVertCoordLow(const float &x, const float &y, const float &z) const {
  float vertAngle = getVertAngle(x, y, z);

  auto it = std::lower_bound(vertAngles.begin(), vertAngles.end(), vertAngle * 180.0 / M_PI);

  // int vertCoord = int((vertRange / 2.0 - vertAngle) / vertRange * (vertRes - 1));
  int vertCoord = vertAngles.end() - it - 1;

  return vertCoord;
}

Eigen::Vector3f VisualView::getDir(const int &r, const int &c) const {
  float horAngle = c * 2 * M_PI / horRes;
  float vertAngle = vertAngles[vertAngles.size() - r - 1] * M_PI / 180.0;

  Eigen::Quaternionf q = Eigen::AngleAxisf(-horAngle, Eigen::Vector3f::UnitZ())
                        * Eigen::AngleAxisf(-vertAngle, Eigen::Vector3f::UnitY());
  Eigen::Vector3f dir = q * Eigen::Vector3f::UnitX();
  return dir;
}

int VisualView::getClosest(const laser_slam::LaserScan &scan,
                           const std::vector<int> &nhs,
                           const Eigen::Vector3f &dir) const {
  int bestNh = -1;
  float bestW = 0.0f;
  for (const int &nh : nhs) {
    if (nh >= 0) {
      float x = scan.scan.features(0, nh);
      float y = scan.scan.features(1, nh);
      float z = scan.scan.features(2, nh);

      Eigen::Vector3f nhDir = Eigen::Vector3f(x, y, z).normalized();
      float curW = dir.dot(nhDir);
      if (curW > bestW) {
        bestNh = nh;
        bestW = curW;
      }
    }
  }
  return bestNh;
}

Eigen::Vector4f VisualView::getPoint(const laser_slam::LaserScan &scan, const int &idx, const int &intDim) const {
  return Eigen::Vector4f(scan.scan.features(0, idx),
                        scan.scan.features(1, idx),
                        scan.scan.features(2, idx),
                        scan.scan.descriptors(intDim, idx));
}


Eigen::Vector4f VisualView::interpolateVert(const Eigen::Vector4f &pt1,
                                            const Eigen::Vector4f &pt2,
                                            const float &vertAngle) const {
  float pt1ang = getVertAngle(pt1(0), pt1(1), pt1(2));
  float pt2ang = getVertAngle(pt2(0), pt2(1), pt2(2));
  float w2 = (vertAngle - pt1ang) / (pt2ang - pt1ang);
  float w1 = (pt2ang - vertAngle) / (pt2ang - pt1ang);

  if (abs(w1 + w2 - 1.0f) > 1e-5) {
    std::cout << "abs(w1 + w2 - 1.0f) > 1e-5" << std::endl;
  }
  if (vertAngle < pt1ang || pt2ang < vertAngle) {
    std::cout << "vertAngle < pt1ang || pt2ang < vertAngle" << std::endl;
  }

  // {
  //   PRINT(pt1.transpose());
  //   PRINT(pt2.transpose());
  //   PRINT(pt1ang);
  //   PRINT(vertAngle);
  //   PRINT(pt2ang);
  //   PRINT(w1);
  //   PRINT(w2);
  //   while (getch() != 'm') {
  //
  //   }
  // }

  return (w1 * pt1 + w2 * pt2) / (w1 + w2);
}

Eigen::Vector4f VisualView::interpolateHor(const Eigen::Vector4f &pt1,
                                           const Eigen::Vector4f &pt2,
                                           const float &horAngle) const {
  float pt1ang = getHorAngle(pt1(0), pt1(1), pt1(2));
  float pt2ang = getHorAngle(pt2(0), pt2(1), pt2(2));
  float w2 = angDiff(horAngle, pt1ang) / angDiff(pt2ang, pt1ang);
  float w1 = angDiff(pt2ang, horAngle) / angDiff(pt2ang, pt1ang);

  // if (abs(w1 + w2 - 1.0f) > 1e-5 || w1 < 0.0f || w2 < 0.0f) {
  //   std::cout << "abs(w1 + w2 - 1.0f) > 1e-5 || w1 < 0.0f || w2 < 0.0f" << std::endl;
  //
  //   {
  //     PRINT(pt1.transpose());
  //     PRINT(pt2.transpose());
  //     PRINT(pt1ang);
  //     PRINT(horAngle);
  //     PRINT(pt2ang);
  //     PRINT(w1);
  //     PRINT(w2);
  //     while (getch() != 'm') {
  //
  //     }
  //   }
  // }
  // if (horAngle < pt1ang || pt2ang < horAngle) {
  //   std::cout << "horAngle < pt1ang || pt2ang < horAngle" << std::endl;
  // }

  return (w1 * pt1 + w2 * pt2) / (w1 + w2);
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
  isCompressedFlag = true;
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
  isCompressedFlag = false;
}


int VisualView::ids = 0;

}