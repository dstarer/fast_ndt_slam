//
// Created by yunle on 18-10-8.
//

#ifndef FAST_NDT_SLAM_LIDARMAPPING_H
#define FAST_NDT_SLAM_LIDARMAPPING_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/registration/ndt.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <time.h>

namespace FAST_NDT {
	using PointT = pcl::PointXYZ;

    struct Pose {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        void init() {
            x = y = z = 0;
            roll = pitch = yaw = 0;
        }
        Eigen::Matrix4f rotateRPY() {
        	Eigen::Translation3f tf_trans(x, y, z);
        	Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
        	Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
        	Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
        	Eigen::Matrix4f mat = (tf_trans * rot_z * rot_y * rot_x).matrix();
        	return mat;
        }
    };
    class LidarMapping {
    public:
        LidarMapping(): globalMapPtr(new pcl::PointCloud<PointT>()) {
            maxIter = 30;
            ndt_res = 1.0;
            step_size = 0.1;
            trans_eps = 0.01;
            voxel_leaf_size = 2.0;

            min_add_scan_shift = 1;
            min_scan_range = 5.0;
            max_scan_range = 200.0;
            map_initialed = false;

            previous_pose.init();
            guess_pose.init();
            current_pose.init();
        }

        void setup(ros::NodeHandle handle, ros::NodeHandle privateHandle);

        void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud);

		protected:
    		/**
    		 * select region map in [min_x, min_y, max_x, max_y]
    		 * **/
    		void update_region_map(double max_x, double max_y, double min_x, double min_y);
    private:
        pcl::PointCloud<PointT>::Ptr globalMapPtr;
        pcl::PointCloud<PointT>::Ptr regionMapPtr;
#ifdef CUDA_FOUND
        gpu::GNormalDistributionsTransform anh_gpu_ndt;
#endif
        pcl::NormalDistributionsTransform<PointT, PointT> ndt;
        Pose previous_pose;
        Pose guess_pose;
        Pose current_pose;
        Pose added_pose;
        int maxIter;
        float ndt_res;
        double step_size;
        double trans_eps;
        double voxel_leaf_size;

        double min_add_scan_shift;
        double min_scan_range;
        double max_scan_range;
        bool map_initialed;

        // publisher
        ros::Publisher current_pose_pub;
        ros::Publisher ndt_map_pub;
        // subscriber
        ros::Subscriber points_sub;
        ros::Subscriber output_sub;
    };
}


#endif //FAST_NDT_SLAM_LIDARMAPPING_H
