/*
    Copyright (c) 2021 Mobile Robots Laboratory at Poznan University of Technology:
    -Jan Wietrzykowski name.surname [at] put.poznan.pl

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef LASER_SLAM_ROS_VISUAL_VIEW_HPP
#define LASER_SLAM_ROS_VISUAL_VIEW_HPP

#include <array>
#include <map>

#include <Eigen/Dense>

#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>

namespace laser_slam_ros {

class VisualView {
public:
  //! A dense matrix over ScalarType
  typedef typename Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix;
  typedef typename Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixInt;

  VisualView();

  VisualView(const laser_slam::LaserScan &iscan,
             const laser_slam::Pose &ipose,
             const int &ihorRes = 1024,
             const int &ivertRes = 64,
             bool iorganized = true);

  // VisualView(VisualView &&other);
  // VisualView(const VisualView &other);

  // ~VisualView();

  void setPose(const laser_slam::Pose &newPose) {
    pose = newPose;
  }

  const laser_slam::Pose &getPose() const {
    return pose;
  }

  const curves::Time &getTime() const {
    return time_ns;
  }

  // MatrixInt getMask(const segmatch::SegmentedCloud &segmentedCloud);

  MatrixInt getMask(const laser_slam_ros::PointCloud &point_cloud) const;

  const Matrix &getIntensity() const {
    return intensity;
  }

  const Matrix &getRange() const {
    return range;
  }

  const MatrixInt &getCount() const {
    return count;
  }

  void compress();

  void decompress();

  bool isCompressed() const {
    return isCompressedFlag;
  }

private:
  float angDiff(const float &a1, const float &a2) const;

  float getHorAngle(const float &x, const float &y, const float &z) const;
  float getVertAngle(const float &x, const float &y, const float &z) const;

  int getHorCoord(const float &x, const float &y, const float &z) const;
  int getVertCoord(const float &x, const float &y, const float &z) const;

  int getHorCoordLow(const float &x, const float &y, const float &z) const;
  int getVertCoordLow(const float &x, const float &y, const float &z) const;

  Eigen::Vector3f getDir(const int &r, const int &c) const;

  int getClosest(const laser_slam::LaserScan &scan,
                  const std::vector<int> &nhs,
                  const Eigen::Vector3f &dir) const;

  Eigen::Vector4f getPoint(const laser_slam::LaserScan &scan, const int &idx, const int &intDim) const;

  Eigen::Vector4f interpolateVert(const Eigen::Vector4f &pt1,
                                  const Eigen::Vector4f &pt2,
                                  const float &vertAngle) const;

  Eigen::Vector4f interpolateHor(const Eigen::Vector4f &pt1,
                                 const Eigen::Vector4f &pt2,
                                 const float &horAngle) const;

  std::pair<int, int> getClosestDir(const float &x, const float &y, const float &z,
                                    const int &r1,
                                    const int &c1,
                                    const int &r2,
                                    const int &c2) const;

  std::vector<uint8_t> compressData(const std::vector<uint8_t> &data);
  std::vector<uint8_t> decompressData(const std::vector<uint8_t> &dataComp);

  // void buildKdtree();

  bool organized;

  int horRes = 1024;
  int vertRes = 64;
  // static constexpr float vertRange = 33.222 * M_PI / 180.0;
  static constexpr float rangeThresh = 1.0;
  static constexpr float occlusionThresh = 1.0;
  static constexpr float dirThresh = 3.0;
  // for OS1 with 1024 horizontal beams
  std::array<int, 4> pixelOffsets;
  std::array<float, 64> vertAngles;

  laser_slam::Pose pose;
  curves::Time time_ns;

  Matrix intensity;
  Matrix range;
  MatrixInt count;

  std::vector<Eigen::Vector3f> dirs;

  bool isCompressedFlag;
  std::vector<uint8_t> intensityComp;
  std::vector<uint8_t> rangeComp;
  std::vector<uint8_t> countComp;
  std::vector<uint8_t> dirsComp;

  static int ids;
};

}

#endif //LASER_SLAM_ROS_VISUAL_VIEW_HPP
