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

        // MatrixInt getMask(const segmatch::SegmentedCloud &segmentedCloud);

        MatrixInt getMask(const laser_slam_ros::PointCloud &point_cloud);

        const Matrix& getIntensity(){
            return intensity;
        }

        const Matrix& getRange(){
            return range;
        }

    private:
        laser_slam::Pose pose;
        curves::Time time_ns;

        Matrix intensity;
        Matrix range;
    };

}

#endif //LASER_SLAM_ROS_VISUAL_VIEW_HPP
