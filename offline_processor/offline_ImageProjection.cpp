#include "offline_ImageProjection.hpp"
#include <pcl/common/transforms.h>
#include "Logger.hpp"

OfflineImageProjection::OfflineImageProjection(int n_scan, int horizon_scan)
	: N_SCAN(n_scan), Horizon_SCAN(horizon_scan), lidarMinRange(1.0), lidarMaxRange(100.0) {
	// Initialize configuration parameters
}

OfflineImageProjection::CloudInfo OfflineImageProjection::processPointCloud(
	const pcl::PointCloud<PointType>::Ptr& laserCloudIn,
	const std::vector<IMUState>& imuData,
	pcl::PointCloud<PointType>::Ptr& deskewedCloud) {
	// Initialize cloud info
	CloudInfo cloudInfo;
	cloudInfo.timestamp
		= laserCloudIn->header.stamp * 1e-6;  // Convert from microseconds to seconds if needed

	// Check if IMU data is available
	cloudInfo.imu_available = !imuData.empty();

	// Initialize output cloud if needed
	if (!deskewedCloud) {
		deskewedCloud.reset(new pcl::PointCloud<PointType>());
	}

	// Simple copy for now - no deskewing
	*deskewedCloud = *laserCloudIn;

	return cloudInfo;
}

OfflineImageProjection::CloudInfo OfflineImageProjection::processPointCloudWithOdom(
	const pcl::PointCloud<PointType>::Ptr& laserCloudIn,
	OfflineProcessor* processor,
	pcl::PointCloud<PointType>::Ptr& deskewedCloud) {
	// Initialize cloud info
	CloudInfo cloudInfo;
	cloudInfo.timestamp = laserCloudIn->header.stamp * 1e-6;
	cloudInfo.odom_available = true;  // We'll set this to false if we have issues

	// Initialize output cloud if needed
	if (!deskewedCloud) {
		deskewedCloud.reset(new pcl::PointCloud<PointType>());
	}
	deskewedCloud->reserve(laserCloudIn->size());

	LOG_INFO("Starting point cloud deskewing with " + std::to_string(laserCloudIn->size()) + " points");

	// Get the base timestamp (scan start time)
	double baseTimestamp = cloudInfo.timestamp;

	// Get initial odometry for the CloudInfo
	OdomData initialOdom = processor->getInterpolatedOdomData(baseTimestamp);

	// Store initial pose in cloud info
	Eigen::Vector3f translation = initialOdom.transform.block<3, 1>(0, 3);
	Eigen::Matrix3f rotation = initialOdom.transform.block<3, 3>(0, 0);

	// Convert rotation matrix to quaternion to avoid gimbal lock
	Eigen::Quaternionf initialQuat(rotation);

	// Initialize reference values if not already set
	if (!reference_set) {
		// Store reference quaternion
		reference_quat = initialQuat;
		reference_set = true;

		// For reporting purposes, extract roll/pitch
		float ref_roll, ref_pitch, ref_yaw;
		extractRPY(rotation, ref_roll, ref_pitch, ref_yaw);
		LOG_INFO("Reference orientation set: roll=" + std::to_string(ref_roll)
				 + ", pitch=" + std::to_string(ref_pitch));
	}

	// Extract yaw from current quaternion (we want to keep the original yaw)
	float roll, pitch, yaw;
	extractRPY(rotation, roll, pitch, yaw);

	// Calculate relative roll and pitch values for reporting
	// Use quaternions for the actual transformations
	cloudInfo.initial_guess_x = translation.x();
	cloudInfo.initial_guess_y = translation.y();
	cloudInfo.initial_guess_z = translation.z();
	cloudInfo.initial_guess_roll = roll;
	cloudInfo.initial_guess_pitch = pitch;
	cloudInfo.initial_guess_yaw = yaw;

	std::cout << "Initial guess: "
			  << "x=" << cloudInfo.initial_guess_x << ", y=" << cloudInfo.initial_guess_y
			  << ", z=" << cloudInfo.initial_guess_z << ", roll=" << cloudInfo.initial_guess_roll
			  << ", pitch=" << cloudInfo.initial_guess_pitch << ", yaw=" << cloudInfo.initial_guess_yaw
			  << std::endl;

	// Define the hood filtering parameters
	const float hood_filter_x = 9.0f;  // 9 meters in x direction
	const float hood_filter_y = 2.5f;  // 2.5 meters in y direction
	const float max_distance = 50.0f;  // 50 meters maximum distance
	int filtered_hood_points = 0;
	int filtered_distance_points = 0;

	// Process each point with time-based deskewing
	// for (const auto& point : laserCloudIn->points) {
	// 	// Filter hood points
	// 	if (std::abs(point.x) < hood_filter_x / 2 && std::abs(point.y) < hood_filter_y / 2 && point.z < 0) {
	// 		filtered_hood_points++;
	// 		continue;  // Skip this point
	// 	}

	// 	// Filter distant points
	// 	float point_distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
	// 	if (point_distance > max_distance) {
	// 		filtered_distance_points++;
	// 		continue;  // Skip this point
	// 	}

	// 	// Get timestamp for this point
	// 	double pointTimestamp = baseTimestamp + point.time;

	// 	// Get odometry data for this specific point time
	// 	OdomData pointOdom = processor->getInterpolatedOdomData(pointTimestamp);

	// 	// Extract rotation as quaternion
	// 	Eigen::Matrix3f pointRotation = pointOdom.transform.block<3, 3>(0, 0);
	// 	Eigen::Quaternionf pointQuat(pointRotation);

	// 	// Extract yaw from point rotation
	// 	float point_roll, point_pitch, point_yaw;
	// 	extractRPY(pointRotation, point_roll, point_pitch, point_yaw);

	// 	// Construct a quaternion for yaw only
	// 	Eigen::Quaternionf yawQuat;
	// 	yawQuat = Eigen::AngleAxisf(point_yaw, Eigen::Vector3f::UnitZ());

	// 	// Extract reference roll and pitch as a quaternion (without yaw)
	// 	float ref_roll, ref_pitch, ref_yaw;
	// 	Eigen::Matrix3f refRotation = reference_quat.toRotationMatrix();
	// 	extractRPY(refRotation, ref_roll, ref_pitch, ref_yaw);

	// 	// Create a quaternion for reference roll and pitch only
	// 	Eigen::Quaternionf rollPitchQuat;
	// 	rollPitchQuat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())
	// 					* Eigen::AngleAxisf(ref_pitch, Eigen::Vector3f::UnitY())
	// 					* Eigen::AngleAxisf(ref_roll, Eigen::Vector3f::UnitX());

	// 	// Combine yaw from current point with reference roll/pitch
	// 	Eigen::Quaternionf combinedQuat = yawQuat * rollPitchQuat;

	// 	// Create modified transform
	// 	Eigen::Matrix4f modifiedTransform = Eigen::Matrix4f::Identity();
	// 	modifiedTransform.block<3, 3>(0, 0) = combinedQuat.toRotationMatrix();
	// 	modifiedTransform.block<3, 1>(0, 3) = pointOdom.transform.block<3, 1>(0, 3);

	// 	// Transform the point
	// 	Eigen::Vector4f pointPos(point.x, point.y, point.z, 1.0f);
	// 	Eigen::Vector4f transformedPos = modifiedTransform * pointPos;

	// 	// Update point and add to cloud
	// 	PointType transformedPoint = point;
	// 	transformedPoint.x = transformedPos.x();
	// 	transformedPoint.y = transformedPos.y();
	// 	transformedPoint.z = transformedPos.z();
	// 	deskewedCloud->push_back(transformedPoint);
	// }
	for (const auto& point : laserCloudIn->points) {
		// Filter out points that might be part of the hood
		// Assuming hood is centered around (0,0) in vehicle frame and extends 2.5m in x and y
		if (std::abs(point.x) < hood_filter_x / 2 && std::abs(point.y) < hood_filter_y / 2 && point.z < 0) {
			filtered_hood_points++;
			continue;  // Skip this point
		}

		// Filter out points that are too far (>50m)
		float point_distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
		if (point_distance > max_distance) {
			filtered_distance_points++;
			continue;  // Skip this point
		}

		// Calculate the exact timestamp for this point
		// The time field in PointXYZIRTRGB is relative to the scan start time
		double pointTimestamp = baseTimestamp + point.time;
		// Get odometry data for this specific point time
		OdomData pointOdom = processor->getInterpolatedOdomData(pointTimestamp);
		// Create a transformed point
		PointType transformedPoint = point;	 // Copy all fields first
		// Transform XYZ coordinates
		Eigen::Vector4f pointPos(point.x, point.y, point.z, 1.0f);
		Eigen::Vector4f transformedPos = pointOdom.transform * pointPos;
		// Update the coordinates
		transformedPoint.x = transformedPos.x();
		transformedPoint.y = transformedPos.y();
		transformedPoint.z = transformedPos.z();
		// Keep other fields (intensity, ring, time, rgb) unchanged
		// Add to the deskewed cloud
		deskewedCloud->push_back(transformedPoint);
	}

	// Set headers and organize the cloud
	deskewedCloud->header = laserCloudIn->header;
	deskewedCloud->height = 1;
	deskewedCloud->width = deskewedCloud->size();
	deskewedCloud->is_dense = laserCloudIn->is_dense;

	LOG_INFO("Completed deskewing, output cloud has " + std::to_string(deskewedCloud->size())
			 + " points (filtered " + std::to_string(filtered_hood_points) + " hood points, "
			 + std::to_string(filtered_distance_points) + " distance points)");

	return cloudInfo;
}
void OfflineImageProjection::setReferencePose(const Eigen::Matrix4f& reference) {
	reference_transform = reference;
	reference_set = true;
	LOG_INFO("Reference pose set for global alignment");
}
void OfflineImageProjection::extractRPY(const Eigen::Matrix3f& rotation,
										float& roll,
										float& pitch,
										float& yaw) {
	// Extract Euler angles from rotation matrix
	// This is a simplified version - you might want to use a more robust method
	pitch = asin(-rotation(2, 0));

	if (cos(pitch) > 1e-10) {
		roll = atan2(rotation(2, 1), rotation(2, 2));
		yaw = atan2(rotation(1, 0), rotation(0, 0));
	} else {
		// Gimbal lock case
		roll = atan2(-rotation(0, 2), rotation(1, 1));
		yaw = 0;
	}
}

// We won't implement these methods for now since we're just doing basic transformation
void OfflineImageProjection::deskewPoint() {
	// Not implemented yet
}

void OfflineImageProjection::projectPointCloud() {
	// Not implemented yet
}

void OfflineImageProjection::cloudExtraction() {
	// Not implemented yet
}
