//
// Created by yunle on 18-10-8.
//
#include "fast_ndt_slam/LidarMapping.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "fast_ndt_mapping");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	FAST_NDT::LidarMapping mapping;
	mapping.setup(nh, private_nh);
	ros::Rate(10);
	ros::spin();
}
