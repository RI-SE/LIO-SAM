#pragma once
#ifndef _OFFLINE_UTILITY_LIDAR_ODOMETRY_H_
#define _OFFLINE_UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

// PCL includes
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>

// OpenCV and Eigen should be after PCL
#include <opencv2/opencv.hpp>

using namespace std;

// Custom point type definition
struct PointXYZIRTRGB {
	PCL_ADD_POINT4D
	PCL_ADD_INTENSITY;
	PCL_ADD_RGB;
	std::uint16_t ring;
	float time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTRGB,
								  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
									  std::uint16_t,
									  ring,
									  ring)(float, time, time)(float, rgb, rgb))

// Use your custom point type
typedef PointXYZIRTRGB PointType;

enum class SensorType { VELODYNE, OUSTER, LIVOX };

struct OdomData {
	double timestamp;
	Eigen::Matrix4f transform;
	Eigen::Vector3f linear_velocity;
	Eigen::Vector3f angular_velocity;
};
// Simple configuration class (not ROS-based)
class OfflineParams {
   public:
	// Topics
	string pointCloudTopic;
	string imuTopic;
	string odomTopic;

	// Frames
	string lidarFrame;
	string odometryFrame;
	string mapFrame;

	bool useDownSampling;

	// Lidar Sensor Configuration
	SensorType sensor = SensorType::VELODYNE;
	int N_SCAN;
	int Horizon_SCAN;
	int downsampleRate;
	float lidarMinRange;
	float lidarMaxRange;

	// LOAM
	float edgeThreshold;
	float surfThreshold;
	int edgeFeatureMinValidNum;
	int surfFeatureMinValidNum;

	// voxel filter params
	float odometrySurfLeafSize;
	float mappingCornerLeafSize;
	float mappingSurfLeafSize;

	float z_tollerance;
	float rotation_tollerance;

	// CPU Params
	int numberOfCores;
	double mappingProcessInterval;

	// Surrounding map
	float surroundingkeyframeAddingDistThreshold;
	float surroundingkeyframeAddingAngleThreshold;
	float surroundingKeyframeDensity;
	float surroundingKeyframeSearchRadius;

	// global map visualization radius
	float globalMapVisualizationSearchRadius;
	float globalMapVisualizationPoseDensity;
	float globalMapVisualizationLeafSize;

	// Constructor with default values
	OfflineParams() {
		// Default values (same as in ParamServer)
		useDownSampling = true;
		pointCloudTopic = "points";
		imuTopic = "imu/data";
		odomTopic = "lio_sam/odometry/imu";

		lidarFrame = "laser_data_frame";
		odometryFrame = "odom";
		mapFrame = "map";

		N_SCAN = 64;
		Horizon_SCAN = 512;
		downsampleRate = 1;
		lidarMinRange = 5.5;
		lidarMaxRange = 1000.0;

		edgeThreshold = 1.0;
		surfThreshold = 0.1;
		edgeFeatureMinValidNum = 10;
		surfFeatureMinValidNum = 100;

		odometrySurfLeafSize = 0.4;
		mappingCornerLeafSize = 0.2;
		mappingSurfLeafSize = 0.4;

		z_tollerance = 1000.0;
		rotation_tollerance = 1000.0;

		numberOfCores = 4;
		mappingProcessInterval = 0.15;

		surroundingkeyframeAddingDistThreshold = 1.0;
		surroundingkeyframeAddingAngleThreshold = 0.2;
		surroundingKeyframeDensity = 2.0;
		surroundingKeyframeSearchRadius = 50.0;

		globalMapVisualizationSearchRadius = 1000.0;
		globalMapVisualizationPoseDensity = 10.0;
		globalMapVisualizationLeafSize = 1.0;
	}

	// Load parameters from a configuration file (could be implemented if needed)
	bool loadFromFile(const std::string& configFile) {
		// Implement if needed
		return true;
	}
};

// Utility function declarations (adapted from original)
template <class T>
float pointDistance(T p) {
	return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <class T>
float pointDistance(T p1, T p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)
				+ (p1.z - p2.z) * (p1.z - p2.z));
}

// Instead of using rclcpp::Time, we use a simple function to convert time
inline double stampToSec(uint32_t sec, uint32_t nanosec) {
	return static_cast<double>(sec) + static_cast<double>(nanosec) * 1e-9;
}

#endif	// _OFFLINE_UTILITY_LIDAR_ODOMETRY_H_