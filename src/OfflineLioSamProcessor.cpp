#include <chrono>
#include <memory>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <unordered_map>
#include <vector>
#include "utility.hpp"

class OfflineLioSamProcessor : public ParamServer {
   private:
	// Data storage
	std::vector<sensor_msgs::msg::PointCloud2> lidarMsgs;
	std::vector<nav_msgs::msg::Odometry> odomMsgs;
	std::vector<sensor_msgs::msg::Imu> imuMsgs;

	// For visualization
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLidarCloud;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubOxtsPath;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarkers;

	// Trajectory
	nav_msgs::msg::Path oxtsTrajectory;

   public:
	OfflineLioSamProcessor(const rclcpp::NodeOptions& options) : ParamServer("offline_lio_sam", options) {
		// Publishers for visualization
		pubLidarCloud = create_publisher<sensor_msgs::msg::PointCloud2>("offline_lio_sam/lidar_cloud", 10);
		pubOxtsPath = create_publisher<nav_msgs::msg::Path>("offline_lio_sam/oxts_path", 10);
		pubMarkers = create_publisher<visualization_msgs::msg::MarkerArray>("offline_lio_sam/markers", 10);

		// Initialize path message
		oxtsTrajectory.header.frame_id = mapFrame;
	}

	bool loadBagFile(const std::string& bagPath) {
		RCLCPP_INFO(get_logger(), "Loading bag file: %s", bagPath.c_str());

		// Configure the reader
		rosbag2_storage::StorageOptions storage_options;
		storage_options.uri = bagPath;
		storage_options.storage_id = "sqlite3";

		rosbag2_cpp::ConverterOptions converter_options;
		converter_options.input_serialization_format = "cdr";
		converter_options.output_serialization_format = "cdr";

		// Create and initialize the reader
		auto reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();

		try {
			reader->open(storage_options, converter_options);
		} catch (const std::exception& e) {
			RCLCPP_ERROR(get_logger(), "Failed to open bag file: %s", e.what());
			return false;
		}

		// Get topic types
		auto topics = reader->get_all_topics_and_types();
		std::unordered_map<std::string, std::string> topic_type_map;
		for (const auto& topic : topics) {
			topic_type_map[topic.name] = topic.type;
			RCLCPP_INFO(get_logger(), "Topic: %s, Type: %s", topic.name.c_str(), topic.type.c_str());
		}

		// Serialization helpers
		rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
		rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
		rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;

		// Read all messages from the bag
		RCLCPP_INFO(get_logger(), "Reading messages from bag...");
		int lidarCount = 0, odomCount = 0, imuCount = 0;

		// For progress reporting
		int processed_messages = 0;
		auto last_report_time = std::chrono::steady_clock::now();

		while (reader->has_next()) {
			auto bag_message = reader->read_next();
			processed_messages++;

			// Show progress every 5 seconds
			auto now = std::chrono::steady_clock::now();
			if (std::chrono::duration_cast<std::chrono::seconds>(now - last_report_time).count() >= 5) {
				RCLCPP_INFO(get_logger(), "Loading progress: %d messages processed", processed_messages);
				last_report_time = now;
			}

			// Process OXTS odometry - specifically the lidar-frame odometry
			if (bag_message->topic_name == "/ins/odometry_lidar") {
				nav_msgs::msg::Odometry odom_msg;
				rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
				odom_serialization.deserialize_message(&serialized_msg, &odom_msg);

				odomMsgs.push_back(odom_msg);
				odomCount++;

				// Add to visualization path
				geometry_msgs::msg::PoseStamped pose_stamped;
				pose_stamped.header = odom_msg.header;
				pose_stamped.pose = odom_msg.pose.pose;
				oxtsTrajectory.poses.push_back(pose_stamped);
			}

			// Process LiDAR messages
			if (bag_message->topic_name == "/velodyne_points") {
				sensor_msgs::msg::PointCloud2 cloud_msg;
				rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
				cloud_serialization.deserialize_message(&serialized_msg, &cloud_msg);

				lidarMsgs.push_back(cloud_msg);
				lidarCount++;
			}

			// Process IMU messages (might be useful for some LIO-SAM components)
			if (bag_message->topic_name == "/ins/imu") {
				sensor_msgs::msg::Imu imu_msg;
				rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
				imu_serialization.deserialize_message(&serialized_msg, &imu_msg);

				imuMsgs.push_back(imu_msg);
				imuCount++;
			}
		}

		RCLCPP_INFO(get_logger(), "Loaded %d LiDAR messages, %d odometry messages, and %d IMU messages",
					lidarCount, odomCount, imuCount);

		// Sort messages by timestamp
		auto compareTime = [](const auto& a, const auto& b) {
			return stamp2Sec(a.header.stamp) < stamp2Sec(b.header.stamp);
		};

		std::sort(lidarMsgs.begin(), lidarMsgs.end(), compareTime);
		std::sort(odomMsgs.begin(), odomMsgs.end(), compareTime);
		std::sort(imuMsgs.begin(), imuMsgs.end(), compareTime);

		return (!lidarMsgs.empty() && !odomMsgs.empty());
	}

	void visualizeTrajectory() {
		if (odomMsgs.empty()) {
			RCLCPP_WARN(get_logger(), "No odometry messages to visualize");
			return;
		}

		RCLCPP_INFO(get_logger(), "Visualizing OXTS trajectory with %zu points", oxtsTrajectory.poses.size());

		oxtsTrajectory.header.stamp = get_clock()->now();
		pubOxtsPath->publish(oxtsTrajectory);

		// Also publish some markers for timestamps
		visualization_msgs::msg::MarkerArray markers;
		int markerId = 0;

		// Add a marker every 5 seconds
		for (size_t i = 0; i < oxtsTrajectory.poses.size(); i += 100) {
			visualization_msgs::msg::Marker marker;
			marker.header.frame_id = mapFrame;
			marker.header.stamp = get_clock()->now();
			marker.ns = "timestamp_markers";
			marker.id = markerId++;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose = oxtsTrajectory.poses[i].pose;
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 0.5;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;
			markers.markers.push_back(marker);
		}

		pubMarkers->publish(markers);
	}

	nav_msgs::msg::Odometry findClosestOdometry(const rclcpp::Time& targetTime) {
		if (odomMsgs.empty()) {
			RCLCPP_ERROR(get_logger(), "No odometry messages available");
			return nav_msgs::msg::Odometry();
		}

		double targetSec = stamp2Sec(targetTime);

		// Find the closest odometry message
		size_t closestIdx = 0;
		double minTimeDiff = std::numeric_limits<double>::max();

		for (size_t i = 0; i < odomMsgs.size(); i++) {
			double odomTime = stamp2Sec(odomMsgs[i].header.stamp);
			double timeDiff = std::abs(odomTime - targetSec);

			if (timeDiff < minTimeDiff) {
				minTimeDiff = timeDiff;
				closestIdx = i;
			}
		}

		RCLCPP_DEBUG(get_logger(), "Found odometry %.3f seconds from target time", minTimeDiff);
		return odomMsgs[closestIdx];
	}

	std::pair<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry> findBracketingOdometry(
		const rclcpp::Time& targetTime) {
		if (odomMsgs.size() < 2) {
			RCLCPP_ERROR(get_logger(), "Not enough odometry messages for interpolation");
			return {nav_msgs::msg::Odometry(), nav_msgs::msg::Odometry()};
		}

		double targetSec = stamp2Sec(targetTime);

		// Find odometry messages that bracket the target time
		size_t beforeIdx = 0;
		size_t afterIdx = 0;
		bool found = false;

		for (size_t i = 0; i < odomMsgs.size() - 1; i++) {
			double beforeTime = stamp2Sec(odomMsgs[i].header.stamp);
			double afterTime = stamp2Sec(odomMsgs[i + 1].header.stamp);

			if (beforeTime <= targetSec && targetSec <= afterTime) {
				beforeIdx = i;
				afterIdx = i + 1;
				found = true;
				break;
			}
		}

		if (!found) {
			RCLCPP_WARN(get_logger(), "Could not bracket time %.3f, using closest", targetSec);
			auto closest = findClosestOdometry(targetTime);
			return {closest, closest};
		}

		return {odomMsgs[beforeIdx], odomMsgs[afterIdx]};
	}

	nav_msgs::msg::Odometry interpolateOdometry(const nav_msgs::msg::Odometry& odom1,
												const nav_msgs::msg::Odometry& odom2,
												double ratio) {
		nav_msgs::msg::Odometry result;
		result.header = odom1.header;  // Use the first header as base

		// Linear interpolation for position
		result.pose.pose.position.x
			= odom1.pose.pose.position.x + ratio * (odom2.pose.pose.position.x - odom1.pose.pose.position.x);
		result.pose.pose.position.y
			= odom1.pose.pose.position.y + ratio * (odom2.pose.pose.position.y - odom1.pose.pose.position.y);
		result.pose.pose.position.z
			= odom1.pose.pose.position.z + ratio * (odom2.pose.pose.position.z - odom1.pose.pose.position.z);

		// SLERP for orientation
		tf2::Quaternion q1, q2, qInterp;
		tf2::fromMsg(odom1.pose.pose.orientation, q1);
		tf2::fromMsg(odom2.pose.pose.orientation, q2);

		// Ensure quaternions are normalized
		q1.normalize();
		q2.normalize();

		// Perform SLERP
		qInterp = q1.slerp(q2, ratio);
		qInterp.normalize();

		// Convert back to geometry_msgs
		result.pose.pose.orientation = tf2::toMsg(qInterp);

		// Linear interpolation for twist
		result.twist.twist.linear.x
			= odom1.twist.twist.linear.x + ratio * (odom2.twist.twist.linear.x - odom1.twist.twist.linear.x);
		result.twist.twist.linear.y
			= odom1.twist.twist.linear.y + ratio * (odom2.twist.twist.linear.y - odom1.twist.twist.linear.y);
		result.twist.twist.linear.z
			= odom1.twist.twist.linear.z + ratio * (odom2.twist.twist.linear.z - odom1.twist.twist.linear.z);

		result.twist.twist.angular.x = odom1.twist.twist.angular.x
									   + ratio * (odom2.twist.twist.angular.x - odom1.twist.twist.angular.x);
		result.twist.twist.angular.y = odom1.twist.twist.angular.y
									   + ratio * (odom2.twist.twist.angular.y - odom1.twist.twist.angular.y);
		result.twist.twist.angular.z = odom1.twist.twist.angular.z
									   + ratio * (odom2.twist.twist.angular.z - odom1.twist.twist.angular.z);

		return result;
	}

	sensor_msgs::msg::Imu findClosestImu(const rclcpp::Time& targetTime) {
		if (imuMsgs.empty()) {
			RCLCPP_ERROR(get_logger(), "No IMU messages available");
			return sensor_msgs::msg::Imu();
		}

		double targetSec = stamp2Sec(targetTime);

		// Find the closest IMU message
		size_t closestIdx = 0;
		double minTimeDiff = std::numeric_limits<double>::max();

		for (size_t i = 0; i < imuMsgs.size(); i++) {
			double imuTime = stamp2Sec(imuMsgs[i].header.stamp);
			double timeDiff = std::abs(imuTime - targetSec);

			if (timeDiff < minTimeDiff) {
				minTimeDiff = timeDiff;
				closestIdx = i;
			}
		}

		RCLCPP_DEBUG(get_logger(), "Found IMU %.3f seconds from target time", minTimeDiff);
		return imuMsgs[closestIdx];
	}

	void processScan(size_t scanIndex) {
		if (scanIndex >= lidarMsgs.size()) {
			RCLCPP_ERROR(get_logger(), "Invalid scan index: %zu", scanIndex);
			return;
		}

		auto& lidarMsg = lidarMsgs[scanIndex];
		auto bracketingOdom = findBracketingOdometry(lidarMsg.header.stamp);

		double lidarTime = stamp2Sec(lidarMsg.header.stamp);
		double beforeTime = stamp2Sec(bracketingOdom.first.header.stamp);
		double afterTime = stamp2Sec(bracketingOdom.second.header.stamp);

		// Calculate interpolation ratio
		double ratio = 0.0;
		if (afterTime > beforeTime) {
			ratio = (lidarTime - beforeTime) / (afterTime - beforeTime);
		}

		// Get interpolated odometry at exact lidar timestamp
		nav_msgs::msg::Odometry interpolatedOdom
			= interpolateOdometry(bracketingOdom.first, bracketingOdom.second, ratio);

		// Set the timestamp to match the lidar
		interpolatedOdom.header.stamp = lidarMsg.header.stamp;

		// Find closest IMU for orientation reference
		sensor_msgs::msg::Imu closestImu = findClosestImu(lidarMsg.header.stamp);

		// For testing: Publish the current LiDAR cloud
		if (scanIndex % 10 == 0) {			   // Publish every 10th scan to avoid overwhelming RViz
			lidarMsg.header.frame_id = "map";  // Set frame ID for visualization
			pubLidarCloud->publish(lidarMsg);
		}

		RCLCPP_INFO(get_logger(), "Processing scan %zu with interpolated odometry", scanIndex);

		// Here you would add code to feed this to your modified LIO-SAM processing pipeline
		// processWithLioSam(lidarMsg, interpolatedOdom, closestImu);
	}

	void processAllScans() {
		RCLCPP_INFO(get_logger(), "Processing all %zu scans...", lidarMsgs.size());

		for (size_t i = 0; i < lidarMsgs.size(); i++) {
			processScan(i);

			// Report progress
			if (i % 100 == 0 || i == lidarMsgs.size() - 1) {
				RCLCPP_INFO(get_logger(), "Processed %zu/%zu scans (%.1f%%)", i + 1, lidarMsgs.size(),
							(i + 1) * 100.0 / lidarMsgs.size());
			}
		}

		RCLCPP_INFO(get_logger(), "Processing complete!");
	}
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	if (argc != 2) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: %s <path_to_bag_file>", argv[0]);
		return 1;
	}

	rclcpp::NodeOptions options;
	auto processor = std::make_shared<OfflineLioSamProcessor>(options);

	if (!processor->loadBagFile(argv[1])) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load bag file");
		return 1;
	}

	// Visualize the trajectory for verification
	processor->visualizeTrajectory();

	// Keep publishing the visualization data
	rclcpp::Rate rate(1);			// 1 Hz
	for (int i = 0; i < 10; i++) {	// Publish a few times to ensure RViz catches it
		processor->visualizeTrajectory();
		rate.sleep();
	}

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
				"Visualization complete. Press Ctrl+C to exit or any key to continue with processing.");

	// Wait for user input
	std::string input;
	std::getline(std::cin, input);

	// Run the processing pipeline
	processor->processAllScans();

	rclcpp::shutdown();
	return 0;
}