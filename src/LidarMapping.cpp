//
// Created by yunle on 18-10-8.
//

#include "../include/fast_ndt_slam/LidarMapping.h"
namespace FAST_NDT {
void LidarMapping::setup(ros::NodeHandle handle, ros::NodeHandle privateHandle) {
	// initial set.
	if (!handle.getParam("tf_x", current_pose.x)) {
		ROS_ERROR("tf_x is not set");
		exit(1);
	}

	if (!handle.getParam("tf_y", current_pose.y)) {
		ROS_ERROR("tf_y is not set");
		exit(1);
	}

	if (!handle.getParam("tf_z", current_pose.z)) {
		ROS_ERROR("tf_z is not set");
		exit(1);
	}

	if (!handle.getParam("tf_roll", current_pose.roll)) {
		ROS_ERROR("tf_roll is not set");
		exit(1);
	}
	if (!handle.getParam("tf_pitch", current_pose.pitch)) {
		ROS_ERROR("tf_pitch is not set");
		exit(1);
	}
	if (!handle.getParam("tf_yaw", current_pose.yaw)) {
		ROS_ERROR("tf_yaw is not set");
		exit(1);
	}

	ROS_INFO("tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw (%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)", current_pose.x, current_pose.y, current_pose.z, current_pose.roll, current_pose.pitch, current_pose.yaw);

	added_pose = current_pose;
	ndt_map_pub = handle.advertise<sensor_msgs::PointCloud2>("/ndt/map", 10000);
	current_pose_pub = handle.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

	points_sub = handle.subscribe("lidar", 100000, &LidarMapping::points_callback, this);
	// output_sub = handle.subscribe("/ndt_mapping_output", 10, &LidarMapping::output_callback, this);

}

void LidarMapping::update_region_map(double max_x, double max_y, double min_x, double min_y) {
	pcl::ConditionAnd<PointT >::Ptr range_cond(new pcl::ConditionAnd<PointT> ());
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE, min_x)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE, min_y)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LE, max_x)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LE, max_y)));
	pcl::ConditionalRemoval<PointT > cond_rem;
	cond_rem.setCondition(range_cond);
	cond_rem.setInputCloud(globalMapPtr);
	cond_rem.setKeepOrganized(true);
	cond_rem.filter(* regionMapPtr);
}

void LidarMapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud) {
	pcl::PointCloud<PointT > scan, tmp;
	pcl::PointCloud<PointT>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr transformed_scan_ptr(new pcl::PointCloud<PointT>());
	PointT p;
	pcl::fromROSMsg(*input_cloud, tmp);
	// filtered illegal point
	for (pcl::PointCloud<PointT>::const_iterator item = tmp.begin(); item != tmp.end(); item ++) {
		p.x = (double) item->x;
		p.y = (double) item->y;
		p.z = (double) item->z;
		double r = sqrt(p.x * p.x + p.y * p.y);
		if (min_scan_range < r && r < max_scan_range) {
			scan.push_back(p);
		}
	}
	// initial map
	if (!map_initialed) {
		pcl::transformPointCloud(scan, *transformed_scan_ptr, current_pose.rotateRPY());
		*globalMapPtr += scan;
		ndt.setInputTarget(globalMapPtr);
		map_initialed = true;
		return;
	}
	// filter
	pcl::VoxelGrid<PointT> voxel_grid_filter;
	voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
	pcl::PointCloud<PointT>::Ptr scan_ptr(new pcl::PointCloud<PointT>(scan));
	voxel_grid_filter.setInputCloud(scan_ptr);
	voxel_grid_filter.filter(*filtered_scan_ptr);

	ndt.setTransformationEpsilon(trans_eps);
	ndt.setMaximumIterations(maxIter);
	ndt.setStepSize(step_size);
	ndt.setResolution(ndt_res);
	ndt.setInputSource(filtered_scan_ptr);

	guess_pose.x = current_pose.x;
	guess_pose.y = current_pose.y;
	guess_pose.z = current_pose.z;
	guess_pose.roll = current_pose.roll;
	guess_pose.pitch = current_pose.pitch;
	guess_pose.yaw = current_pose.yaw;

	Eigen::Matrix4f init_guess = guess_pose.rotateRPY();

	double t1 = ros::Time::now().toNSec();
	pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>());
	ndt.align(*output_cloud, init_guess);
	double fitness_score = ndt.getFitnessScore();
	Eigen::Matrix4f finalTrans = ndt.getFinalTransformation();
	bool has_converged = ndt.hasConverged();
	int final_num_iteration = ndt.getFinalNumIteration();
//	double transformation_probability = ndt.getTransformationProbability();
	pcl::transformPointCloud(scan, *transformed_scan_ptr, finalTrans);

	double t2 = ros::Time::now().toNSec();

	tf::Matrix3x3 mat_l;
	mat_l.setValue(static_cast<double >(finalTrans(0, 0)), static_cast<double >(finalTrans(0, 1)), static_cast<double >(finalTrans(0, 2)),
								 static_cast<double >(finalTrans(1, 0)), static_cast<double >(finalTrans(1, 1)), static_cast<double >(finalTrans(1, 2)),
								 static_cast<double >(finalTrans(2, 0)), static_cast<double >(finalTrans(2, 1)), static_cast<double >(finalTrans(2, 2)));
	double _tx = finalTrans(0, 3);
	double _ty = finalTrans(1, 3);
	double _tz = finalTrans(2, 3);
	Pose _current_pose;
	_current_pose.init();
	_current_pose.x = _tx;
	_current_pose.y = _ty;
	_current_pose.z = _tz;
	mat_l.getRPY(_current_pose.roll, _current_pose.pitch, _current_pose.yaw);

	double shift = sqrt(pow(current_pose.x - _tx, 2.0) + pow(current_pose.y - _ty, 2.0));
	if (shift >= min_add_scan_shift) { // update the map
		*globalMapPtr += *transformed_scan_ptr;
		added_pose = _current_pose;
		ndt.setInputTarget(globalMapPtr);
	}
	previous_pose = current_pose;
	current_pose = _current_pose;
	ROS_INFO("sequence_number: %d", input_cloud->header.seq);
	ROS_INFO("used %.4f ms", (t2 - t1) / 1000000.);
	ROS_INFO("Number of Scan Points: %u", scan.size());
	ROS_INFO("Number of filtered scan points: %u", filtered_scan_ptr->size());
	ROS_INFO("transformed_scan_ptr: %u", transformed_scan_ptr->size());
	ROS_INFO("map: %u", globalMapPtr->size() );
	ROS_INFO("NDT has converged %u", has_converged);
	ROS_INFO("Fitness score: %u", fitness_score);
	ROS_INFO("Number of Iterations %d", final_num_iteration);
	ROS_INFO("(x, y, z, roll, pitch, yaw): (%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)", current_pose.x, current_pose.y, current_pose.z, current_pose.roll, current_pose.pitch, current_pose.yaw);
	ROS_INFO("Shift %.4f", shift);
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = current_pose.x;
	pose.pose.position.y = current_pose.y;
	pose.pose.position.z = current_pose.z;
	tf::Quaternion q;
	q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
	pose.pose.orientation.x = q.x();
	pose.pose.orientation.y = q.y();
	pose.pose.orientation.z = q.z();
	pose.pose.orientation.w = q.w();
	current_pose_pub.publish(pose);
}
}