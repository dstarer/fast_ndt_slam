//
// Created by yunle on 18-10-8.
//

#ifndef FAST_NDT_SLAM_LIDARMAPPING_H
#define FAST_NDT_SLAM_LIDARMAPPING_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/registration/ndt.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <time.h>

namespace FAST_NDT {
    struct Pose {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };
    class LidarMapping {
    public:
        LidarMapping() {

        }

        void setup(ros::NodeHandle handle, ros::NodeHandle privateHandle);

        void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud);

    private:
        pcl::PointCloud<pcl::PointXYZI>::Ptr globalMapPtr;
        Pose previous_pose;
        Pose guess_pose;
        Pose current_pose;
    };
}


#endif //FAST_NDT_SLAM_LIDARMAPPING_H
