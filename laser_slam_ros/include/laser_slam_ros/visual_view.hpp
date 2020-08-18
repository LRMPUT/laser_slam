//
// Created by janw on 02.06.2020.
//

#ifndef LASER_SLAM_ROS_VISUAL_VIEW_HPP
#define LASER_SLAM_ROS_VISUAL_VIEW_HPP

#include <array>

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

  VisualView(const laser_slam::LaserScan &iscan, const laser_slam::Pose &ipose);

  // VisualView(VisualView &&other);
  // VisualView(const VisualView &other);

  // ~VisualView();

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

private:
  float getHorAngle(const float &x, const float &y, const float &z) const;
  float getVertAngle(const float &x, const float &y, const float &z) const;

  int getHorCoord(const float &x, const float &y, const float &z) const;
  int getVertCoord(const float &x, const float &y, const float &z) const;

  std::pair<int, int> getClosestDir(const float &x, const float &y, const float &z,
                                    const int &r1,
                                    const int &c1,
                                    const int &r2,
                                    const int &c2) const;

  std::vector<uint8_t> compressData(const std::vector<uint8_t> &data);
  std::vector<uint8_t> decompressData(const std::vector<uint8_t> &dataComp);

  // TODO move to params or detect
  static constexpr int horRes = 1024;
  static constexpr int vertRes = 64;
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

  bool isCompressed;
  std::vector<uint8_t> intensityComp;
  std::vector<uint8_t> rangeComp;
  std::vector<uint8_t> countComp;
  std::vector<uint8_t> dirsComp;

  static int ids;
};

}

#endif //LASER_SLAM_ROS_VISUAL_VIEW_HPP
