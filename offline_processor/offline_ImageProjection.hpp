#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>
#include <vector>
#include "offline_Processor.hpp"
#include "offline_utility.hpp"

class OfflineProcessor;
// Core definitions without ROS dependencies
class OfflineImageProjection {
   public:
	struct IMUState {
		double time;
		Eigen::Vector3d angular_velocity;
		Eigen::Vector3d linear_acceleration;
		Eigen::Quaterniond orientation;
	};

	struct CloudInfo {
		double timestamp;
		std::vector<int> start_ring_index;
		std::vector<int> end_ring_index;
		std::vector<int> point_col_ind;
		std::vector<float> point_range;
		bool imu_available = false;
		bool odom_available = false;
		float imu_roll_init, imu_pitch_init, imu_yaw_init;
		float initial_guess_x, initial_guess_y, initial_guess_z;
		float initial_guess_roll, initial_guess_pitch, initial_guess_yaw;
	};
	void setReferencePose(const Eigen::Matrix4f& reference);

	OfflineImageProjection(int n_scan = 64, int horizon_scan = 1800);

	// Process a single scan with IMU data
	CloudInfo processPointCloud(const pcl::PointCloud<PointType>::Ptr& laserCloudIn,
								const std::vector<IMUState>& imuData,
								pcl::PointCloud<PointType>::Ptr& deskewedCloud);

	// Process a single scan with odometry data
	CloudInfo processPointCloudWithOdom(const pcl::PointCloud<PointType>::Ptr& laserCloudIn,
										OfflineProcessor* processor,
										pcl::PointCloud<PointType>::Ptr& deskewedCloud);

   private:
	// Configuration
	int N_SCAN;
	int Horizon_SCAN;
	float lidarMinRange;
	float lidarMaxRange;
	Eigen::Matrix4f reference_transform = Eigen::Matrix4f::Identity();
	float reference_roll = 0.0f;
	float reference_pitch = 0.0f;
	Eigen::Quaternionf reference_quat;
	bool reference_set = false;

	// Helper method to extract roll, pitch, yaw from rotation matrix
	void extractRPY(const Eigen::Matrix3f& rotation, float& roll, float& pitch, float& yaw);

	// Core algorithm methods (to be implemented)
	void deskewPoint();
	void projectPointCloud();
	void cloudExtraction();

	// Other internal methods
};