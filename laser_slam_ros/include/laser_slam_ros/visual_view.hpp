//
// Created by janw on 02.06.2020.
//

#ifndef LASER_SLAM_ROS_VISUAL_VIEW_HPP
#define LASER_SLAM_ROS_VISUAL_VIEW_HPP

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

        const laser_slam::Pose& getPose() const {
          return pose;
        }

        const curves::Time& getTime() const {
          return time_ns;
        }

        // MatrixInt getMask(const segmatch::SegmentedCloud &segmentedCloud);

        MatrixInt getMask(const laser_slam_ros::PointCloud &point_cloud);

        const Matrix& getIntensity() const {
          return intensity;
        }

        const Matrix& getRange() const {
          return range;
        }

        const MatrixInt& getCount() const {
          return count;
        }

    private:
        laser_slam::Pose pose;
        curves::Time time_ns;

        Matrix intensity;
        Matrix range;
        MatrixInt count;
    };

}

#endif //LASER_SLAM_ROS_VISUAL_VIEW_HPP
