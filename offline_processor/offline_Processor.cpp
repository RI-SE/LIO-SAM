#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "Logger.hpp"
#include "offline_FeatureExtraction.hpp"
#include "offline_MapOptimization.hpp"
#include "offline_Processor.hpp"

OfflineProcessor::OfflineProcessor(const Config& config) : config(config) {
	// Create output directory if it doesn't exist
	std::filesystem::create_directories(config.outputDirectory);

	// Initialize processing components with appropriate config
	imageProjection = std::make_unique<OfflineImageProjection>(config.nScan, config.horizonScan);

	featureExtraction
		= std::make_unique<OfflineFeatureExtraction>(config.edgeThreshold, config.surfThreshold);

	mapOptimization
		= std::make_unique<OfflineMapOptimization>(config.mappingCornerLeafSize, config.mappingSurfLeafSize,
												   50.0);  // surroundingKeyframeSearchRadius

	LOG_INFO("Initialized Offline Processor with:");
	LOG_INFO("  - Input bag: " + config.inputBagPath);
	LOG_INFO("  - Output directory: " + config.outputDirectory);
	LOG_INFO("  - High quality mode: " + std::string(config.highQualityMode ? "enabled" : "disabled"));
	LOG_INFO("  - Map resolution: " + std::to_string(config.mapResolution));
}

OfflineProcessor::~OfflineProcessor() {
	// Clean up as needed
}

bool OfflineProcessor::loadBagFile() {
	LOG_INFO("Loading bag file: " + config.inputBagPath);

	// Configure rosbag2 reader
	rosbag2_storage::StorageOptions storage_options;
	storage_options.uri = config.inputBagPath;
	storage_options.storage_id = "sqlite3";

	rosbag2_cpp::ConverterOptions converter_options;
	converter_options.input_serialization_format = "cdr";
	converter_options.output_serialization_format = "cdr";

	auto reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();

	try {
		reader->open(storage_options, converter_options);
	} catch (const std::exception& e) {
		LOG_ERROR(std::string("Failed to open bag file: ") + e.what());
		return false;
	}

	int totalMessages = 0;
	try {
		auto metadata = reader->get_metadata();
		totalMessages = metadata.message_count;
		LOG_INFO("Bag file contains " + std::to_string(totalMessages) + " messages");
	} catch (const std::exception& e) {
		LOG_WARNING("Could not get message count from metadata: " + std::string(e.what()));
		LOG_INFO("Processing without knowing total messages");
	}
	// Create serialization objects
	rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloudSerialization;
	rclcpp::Serialization<sensor_msgs::msg::Imu> imuSerialization;
	rclcpp::Serialization<nav_msgs::msg::Odometry> odomSerialization;

	// Read messages and store them in our data structures
	int processedMessages = 0;
	int lidarCount = 0, imuCount = 0, odomCount = 0;
	auto startTime = std::chrono::steady_clock::now();

	while (reader->has_next()) {
		auto bag_message = reader->read_next();
		processedMessages++;

		// Show progress
		if (processedMessages % 2000 == 0) {
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
			float percentage = (float)processedMessages / totalMessages * 100.0f;
			std::stringstream loadingMsg;
			loadingMsg << "Loading: " << percentage << "% (" << processedMessages << "/" << totalMessages
					   << ") - Elapsed: " << elapsed << "s";
			LOG_INFO(loadingMsg.str());
		}

		// Process LiDAR messages
		if (bag_message->topic_name == "/velodyne_points") {
			sensor_msgs::msg::PointCloud2 cloudMsg;
			rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
			cloudSerialization.deserialize_message(&serialized_msg, &cloudMsg);

			// Convert to PCL cloud
			pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
			pcl::fromROSMsg(cloudMsg, *cloud);

			// Store in our format
			LidarFrame frame;
			frame.timestamp = cloudMsg.header.stamp.sec + cloudMsg.header.stamp.nanosec * 1e-9;
			frame.cloud = cloud;
			// Log every 100th message
			if (lidarCount % 100 == 0) {
				std::stringstream ss;
				ss << "LiDAR #" << lidarCount << " t=" << frame.timestamp << " points=" << cloud->size()
				   << " height=" << cloudMsg.height << " width=" << cloudMsg.width << "\n";

				// Display a few sample points (first 5 points)
				ss << "    Sample points:\n";
				for (size_t i = 0; i < std::min(size_t(5), cloud->size()); i++) {
					const auto& pt = cloud->points[i];
					ss << "      [" << i << "] x=" << pt.x << ", y=" << pt.y << ", z=" << pt.z
					   << ", intensity=" << pt.intensity << ", ring=" << pt.ring << ", time=" << pt.time
					   << ", rgb=" << std::hex << static_cast<int>(pt.rgb) << std::dec << "\n";
				}

				LOG_INFO(ss.str());
			}
			if (lidarCount % 1000 == 0) {
				std::stringstream ss;
				ss << "LiDAR #" << lidarCount << " t=" << frame.timestamp << " points=" << cloud->size()
				   << " height=" << cloudMsg.height << " width=" << cloudMsg.width << "\n";

				// Analyze time values in the cloud
				float minTime = std::numeric_limits<float>::max();
				float maxTime = std::numeric_limits<float>::lowest();
				std::vector<float> timeValues;

				// Collect all time values
				for (size_t i = 0; i < cloud->size(); i++) {
					float t = cloud->points[i].time;
					if (t < minTime)
						minTime = t;
					if (t > maxTime)
						maxTime = t;

					// Store some sample time values
					if (i % (cloud->size() / 20) == 0) {  // Sample ~20 points throughout the cloud
						timeValues.push_back(t);
					}
				}

				// Display time range
				ss << "    Time range: [" << minTime << " to " << maxTime
				   << "], span: " << (maxTime - minTime) << " seconds\n";

				// Display sample time distribution
				ss << "    Time samples (across cloud): ";
				for (float t : timeValues) {
					ss << t << ", ";
				}
				ss << "\n";

				// Display a few sample points with all fields
				ss << "    Sample points (with all fields):\n";
				for (size_t i = 0; i < std::min(size_t(10), cloud->size()); i++) {
					const auto& pt = cloud->points[i];
					ss << "      [" << i << "] x=" << pt.x << ", y=" << pt.y << ", z=" << pt.z
					   << ", intensity=" << pt.intensity << ", ring=" << pt.ring << ", time=" << pt.time
					   << ", rgb=" << std::hex << static_cast<int>(pt.rgb) << std::dec << "\n";
				}

				// Also sample points from different parts of the cloud to see timing patterns
				size_t middle = cloud->size() / 2;
				size_t end = cloud->size() - 1;

				ss << "    Middle point [" << middle << "]: time=" << cloud->points[middle].time
				   << ", ring=" << cloud->points[middle].ring << "\n";
				ss << "    End point [" << end << "]: time=" << cloud->points[end].time
				   << ", ring=" << cloud->points[end].ring << "\n";

				LOG_INFO(ss.str());
			}
			lidarFrames.push_back(frame);
			lidarCount++;
			if (config.maxLidarMessages > 0 && lidarCount >= config.maxLidarMessages) {
				LOG_INFO("Reached maximum LiDAR message count (" + std::to_string(config.maxLidarMessages)
						 + "). Stopping data loading.");
				break;
			}
		}

		// Process IMU messages
		else if (bag_message->topic_name == "/ins/imu") {
			sensor_msgs::msg::Imu imuMsg;
			rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
			imuSerialization.deserialize_message(&serialized_msg, &imuMsg);

			// Convert quaternion to Euler angles in degrees
			double roll, pitch, yaw;

			// Quaternion components
			double w = imuMsg.orientation.w;
			double x = imuMsg.orientation.x;
			double y = imuMsg.orientation.y;
			double z = imuMsg.orientation.z;

			// Roll (x-axis rotation)
			double sinr_cosp = 2.0 * (w * x + y * z);
			double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
			roll = std::atan2(sinr_cosp, cosr_cosp);

			// Pitch (y-axis rotation)
			double sinp = 2.0 * (w * y - z * x);
			if (std::abs(sinp) >= 1)
				pitch = std::copysign(M_PI / 2, sinp);	// use 90 degrees if out of range
			else
				pitch = std::asin(sinp);

			// Yaw (z-axis rotation)
			double siny_cosp = 2.0 * (w * z + x * y);
			double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
			yaw = std::atan2(siny_cosp, cosy_cosp);

			// Convert to degrees
			roll = roll * 180.0 / M_PI;
			pitch = pitch * 180.0 / M_PI;
			yaw = yaw * 180.0 / M_PI;

			// Print all imuMsg data
			// std::cout << "IMU Message #" << imuCount << ":" << std::endl;
			// std::cout << "  Header: " << std::endl;
			// std::cout << "    stamp: " << imuMsg.header.stamp.sec << "." << imuMsg.header.stamp.nanosec
			// 		  << " seconds" << std::endl;
			// std::cout << "    frame_id: " << imuMsg.header.frame_id << std::endl;
			// std::cout << "  Orientation (quaternion): [w=" << imuMsg.orientation.w
			// 		  << ", x=" << imuMsg.orientation.x << ", y=" << imuMsg.orientation.y
			// 		  << ", z=" << imuMsg.orientation.z << "]" << std::endl;
			// std::cout << "  Orientation (degrees): [roll=" << roll << "°, pitch=" << pitch << "°, yaw=" <<
			// yaw
			// 		  << "°]" << std::endl;
			// std::cout << "  Angular velocity: [x=" << imuMsg.angular_velocity.x
			// 		  << ", y=" << imuMsg.angular_velocity.y << ", z=" << imuMsg.angular_velocity.z << "]"
			// 		  << std::endl;
			// std::cout << "  Linear acceleration: [x=" << imuMsg.linear_acceleration.x
			// 		  << ", y=" << imuMsg.linear_acceleration.y << ", z=" << imuMsg.linear_acceleration.z
			// 		  << "]" << std::endl;
			// std::cout << "----------------------------------" << std::endl;

			// Rest of your code remains the same
			// Store in our format
			ImuData data;
			data.timestamp = imuMsg.header.stamp.sec + imuMsg.header.stamp.nanosec * 1e-9;
			data.angular_velocity.x() = imuMsg.angular_velocity.x;
			data.angular_velocity.y() = imuMsg.angular_velocity.y;
			data.angular_velocity.z() = imuMsg.angular_velocity.z;
			data.linear_acceleration.x() = imuMsg.linear_acceleration.x;
			data.linear_acceleration.y() = imuMsg.linear_acceleration.y;
			data.linear_acceleration.z() = imuMsg.linear_acceleration.z;
			data.orientation.w() = imuMsg.orientation.w;
			data.orientation.x() = imuMsg.orientation.x;
			data.orientation.y() = imuMsg.orientation.y;
			data.orientation.z() = imuMsg.orientation.z;
			if (imuCount % 100 == 0) {
				std::stringstream ss;
				ss << "IMU #" << imuCount << " t=" << data.timestamp << " acc=["
				   << data.linear_acceleration.norm() << " m/s²]"
				   << " gyro=[" << data.angular_velocity.norm() << " rad/s]";
				LOG_INFO(ss.str());
			}
			imuData.push_back(data);
			imuCount++;
		}

		// Process Odometry messages
		else if (bag_message->topic_name == "/ins/odometry_lidar") {
			nav_msgs::msg::Odometry odomMsg;
			rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
			odomSerialization.deserialize_message(&serialized_msg, &odomMsg);

			// Store in our format
			OdomData data;
			data.timestamp = odomMsg.header.stamp.sec + odomMsg.header.stamp.nanosec * 1e-9;

			// Convert pose to Eigen matrix
			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

			// // Extract rotation
			Eigen::Quaternionf q(odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x,
								 odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z);
			transform.block<3, 3>(0, 0) = q.toRotationMatrix();

			// // only yaw component
			// Eigen::Matrix3f rotMatrix = q.toRotationMatrix();
			// double yaw = std::atan2(rotMatrix(1, 0), rotMatrix(0, 0));

			// // Create a new rotation matrix with only yaw rotation (around Z-axis)
			// Eigen::Matrix3f yawOnlyRotation
			// 	= Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

			// // Apply only the yaw rotation to the transform
			// transform.block<3, 3>(0, 0) = yawOnlyRotation;
			// Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
			// float yaw = euler[0];	  // Keep yaw as is
			// float pitch = -euler[1];  // Flip the sign of pitch
			// float roll = -euler[2];	  // Flip the sign of roll

			// // Create a new rotation matrix with the modified angles
			// Eigen::Matrix3f modifiedRotation;
			// modifiedRotation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
			// 				   * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
			// 				   * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

			// // Apply the modified rotation to the transform
			// transform.block<3, 3>(0, 0) = modifiedRotation;

			// Extract translation
			transform(0, 3) = odomMsg.pose.pose.position.x;
			transform(1, 3) = odomMsg.pose.pose.position.y;
			transform(2, 3) = odomMsg.pose.pose.position.z;
			data.linear_velocity = Eigen::Vector3f(odomMsg.twist.twist.linear.x, odomMsg.twist.twist.linear.y,
												   odomMsg.twist.twist.linear.z);
			data.angular_velocity = Eigen::Vector3f(
				odomMsg.twist.twist.angular.x, odomMsg.twist.twist.angular.y, odomMsg.twist.twist.angular.z);

			data.transform = transform;
			if (odomCount % 100 == 0) {
				std::stringstream ss;
				ss << "Odom #" << odomCount << " t=" << data.timestamp << " pos=[" << transform(0, 3) << ","
				   << transform(1, 3) << "," << transform(2, 3) << "]"
				   << " rot=[" << transform(0, 0) << "," << transform(1, 0) << "," << transform(2, 0) << "]"
				   << " vel=[" << data.linear_velocity(0) << "," << data.linear_velocity(1) << ","
				   << data.linear_velocity(2) << "]"
				   << " ang=[" << data.angular_velocity(0) << "," << data.angular_velocity(1) << ","
				   << data.angular_velocity(2) << "]";
				LOG_INFO(ss.str());
			}
			odomData.push_back(data);
			odomCount++;
		}
	}
	LOG_INFO("\nLoaded:");
	LOG_INFO("  - " + std::to_string(lidarCount) + " LiDAR frames");
	LOG_INFO("  - " + std::to_string(imuCount) + " IMU messages");
	LOG_INFO("  - " + std::to_string(odomCount) + " Odometry messages");

	// Sort data by timestamp
	std::sort(lidarFrames.begin(), lidarFrames.end(),
			  [](const LidarFrame& a, const LidarFrame& b) { return a.timestamp < b.timestamp; });

	std::sort(imuData.begin(), imuData.end(),
			  [](const ImuData& a, const ImuData& b) { return a.timestamp < b.timestamp; });

	std::sort(odomData.begin(), odomData.end(),
			  [](const OdomData& a, const OdomData& b) { return a.timestamp < b.timestamp; });

	// Verify time ranges
	if (!lidarFrames.empty() && !odomData.empty()) {
		double lidar_start = lidarFrames.front().timestamp;
		double lidar_end = lidarFrames.back().timestamp;
		double odom_start = odomData.front().timestamp;
		double odom_end = odomData.back().timestamp;

		LOG_INFO("Time ranges:");
		std::cout << "  - LiDAR: " << lidar_start << " to " << lidar_end << " (" << (lidar_end - lidar_start)
				  << " seconds)" << std::endl;
		std::cout << "  - Odometry: " << odom_start << " to " << odom_end << " (" << (odom_end - odom_start)
				  << " seconds)" << std::endl;

		if (odom_start > lidar_start || odom_end < lidar_end) {
			LOG_WARNING("WARNING: Odometry data doesn't fully cover LiDAR data!");
		}
	}

	return (!lidarFrames.empty() && !odomData.empty());
}

bool OfflineProcessor::loadPcapFile() {
	LOG_INFO("Loading PCAP file: " + config.inputPcapPath);

	// Validate input file path
	if (!std::filesystem::exists(config.inputPcapPath)) {
		LOG_ERROR("PCAP file does not exist: " + config.inputPcapPath);
		return false;
	}

	// Create Velodyne processor
	VelodyneProcessor velodyneProcessor(config.inputPcapPath,
										"195.0.0.70",  // Velodyne IP
										2368,		   // Velodyne port
										true,		   // Read once
										config.readFast);

	if (!velodyneProcessor.open()) {
		LOG_ERROR("Failed to open PCAP file for Velodyne data");
		return false;
	}

	// Load calibration from a file (create it or specify a path)
	// Load calibration directly from the existing YAML file
	std::string calibrationFile
		= "/home/adam/ros_workspaces/ws_datalog/install/velodyne_pointcloud/share/velodyne_pointcloud/params/"
		  "VeloView-VLP-32C.yaml";

	if (!velodyneProcessor.loadCalibration(calibrationFile)) {
		LOG_WARNING("Could not load calibration file, using default values");
	}

	// Open a second PCAP reader for OXTS data
	pcap_t* oxts_pcap;
	char errbuf[PCAP_ERRBUF_SIZE];
	oxts_pcap = pcap_open_offline(config.inputPcapPath.c_str(), errbuf);

	if (oxts_pcap == NULL) {
		LOG_ERROR(std::string("Failed to open PCAP file for OXTS data: ") + errbuf);
		// Continue with just LiDAR data if PCAP fails
	}

	// Initialize counters
	int lidarCount = 0;
	int imuCount = 0;
	int gpsCount = 0;

	// Initialize OXTS decoder
#ifdef HAVE_NCOMRXC
	NComRxC* ncomDecoder = NComCreateNComRxC();
	if (ncomDecoder == NULL) {
		LOG_WARNING("Failed to create NComRxC decoder");
	}
#endif

	// Process Velodyne packets
	std::vector<uint8_t> packet;
	double packetTime;
	uint16_t lastAzimuth = 0;
	std::shared_ptr<pcl::PointCloud<PointType>> currentScan(new pcl::PointCloud<PointType>());
	double scanStartTime = 0;
	bool firstPacket = true;

	while (velodyneProcessor.getPacket(packet, packetTime)) {
		// Check if this marks a new scan
		bool isNewScan = velodyneProcessor.isNewScan(packet, lastAzimuth);
		if (isNewScan && !currentScan->empty() && !firstPacket) {
			// Save the completed scan
			LidarFrame frame;
			frame.timestamp = scanStartTime;
			frame.cloud = currentScan;

			if (lidarCount % 100 == 0) {
				std::stringstream ss;
				ss << "LiDAR #" << lidarCount << " t=" << frame.timestamp
				   << " points=" << currentScan->size();
				LOG_INFO(ss.str());
			}

			lidarFrames.push_back(frame);
			lidarCount++;

			// Start a new scan
			currentScan.reset(new pcl::PointCloud<PointType>());
		}

		if (isNewScan || firstPacket) {
			scanStartTime = packetTime;
			firstPacket = false;
		}

		// Convert the packet to points
		auto points = velodyneProcessor.convertPacketToPoints(packet, packetTime);

		// Add points to current scan
		for (const auto& point : *points) {
			PointType ourPoint;
			ourPoint.x = point.x;
			ourPoint.y = point.y;
			ourPoint.z = point.z;
			ourPoint.intensity = point.intensity;
			ourPoint.ring = point.ring;
			ourPoint.time = point.time;
			currentScan->push_back(ourPoint);
		}

		if (config.maxLidarMessages > 0 && lidarCount >= config.maxLidarMessages) {
			LOG_INFO("Reached maximum LiDAR message count. Stopping data loading.");
			break;
		}
	}

	// Save the last scan if not empty
	if (!currentScan->empty()) {
		LidarFrame frame;
		frame.timestamp = scanStartTime;
		frame.cloud = currentScan;
		lidarFrames.push_back(frame);
		lidarCount++;
	}

	LOG_INFO("Total complete LiDAR frames: " + std::to_string(lidarFrames.size()));

	// Process OXTS data if we have the PCAP open
	if (oxts_pcap) {
		// Define OXTS IP in network byte order
		in_addr_t oxtsIP = inet_addr("195.0.0.79");

		const u_char* packet;
		struct pcap_pkthdr header;

		while ((packet = pcap_next(oxts_pcap, &header)) != NULL) {
			// Check for Ethernet packets
			struct ether_header* ethHeader = (struct ether_header*)packet;
			if (ntohs(ethHeader->ether_type) != ETHERTYPE_IP) {
				continue;
			}

			// Get IP header
			const struct ip* ipHeader = (struct ip*)(packet + sizeof(struct ether_header));
			if (ipHeader->ip_p != IPPROTO_UDP) {
				continue;
			}

			// Check if it's from OXTS IP
			if (ipHeader->ip_src.s_addr != oxtsIP) {
				continue;
			}

			// Get UDP header
			const struct udphdr* udpHeader
				= (struct udphdr*)(packet + sizeof(struct ether_header) + (ipHeader->ip_hl << 2));

			// Check for OXTS port (3000)
			uint16_t oxtsPort = ntohs(udpHeader->dest);
			if (oxtsPort != 3000) {
				continue;
			}

			// Extract packet timestamp
			double packetTimestamp = header.ts.tv_sec + header.ts.tv_usec * 1e-6;

			// Get payload
			const uint8_t* payload
				= packet + sizeof(struct ether_header) + (ipHeader->ip_hl << 2) + sizeof(struct udphdr);
			int payloadSize = ntohs(udpHeader->len) - sizeof(struct udphdr);

#ifdef HAVE_NCOMRXC
			// Process OXTS data with NComRxC
			if (ncomDecoder) {
				for (int i = 0; i < payloadSize; i++) {
					if (NComNewChar(ncomDecoder, payload[i])) {
						// Process IMU and Nav data as before
						// [Your existing OXTS processing code]
					}
				}
			}
#endif
		}

		pcap_close(oxts_pcap);
	}

#ifdef HAVE_NCOMRXC
	if (ncomDecoder != NULL) {
		NComDestroyNComRxC(ncomDecoder);
	}
#endif

	LOG_INFO("\nLoaded:");
	LOG_INFO("  - " + std::to_string(lidarFrames.size()) + " LiDAR frames");
	LOG_INFO("  - " + std::to_string(imuData.size()) + " IMU messages");
	LOG_INFO("  - " + std::to_string(odomData.size()) + " GPS/Nav messages");

	// Sort data by timestamp
	std::sort(lidarFrames.begin(), lidarFrames.end(),
			  [](const LidarFrame& a, const LidarFrame& b) { return a.timestamp < b.timestamp; });

	std::sort(imuData.begin(), imuData.end(),
			  [](const ImuData& a, const ImuData& b) { return a.timestamp < b.timestamp; });

	std::sort(odomData.begin(), odomData.end(),
			  [](const OdomData& a, const OdomData& b) { return a.timestamp < b.timestamp; });

	return (!lidarFrames.empty());
}
// Helper function to process Velodyne packets into a point cloud
pcl::PointCloud<PointType>::Ptr OfflineProcessor::processVelodynePackets(
	const std::vector<std::vector<uint8_t>>& packets) {
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

	// Implementation of Velodyne packet decoding
	// This would require knowledge of the specific Velodyne model and its packet format
	// Basic outline:
	// 1. For each packet:
	//    a. Extract firing data blocks
	//    b. For each firing:
	//       i. Extract distance, intensity, azimuth, etc.
	//       ii. Convert to cartesian coordinates
	//       iii. Add to point cloud

	// Example simplified implementation for HDL-64E format
	for (const auto& packet : packets) {
		if (packet.size() < 1206)
			continue;  // Minimum size for a valid packet

		// Extract azimuth value (rotation angle)
		for (int block = 0; block < 12; block++) {
			int blockOffset = 100 + block * 100;
			if (blockOffset + 2 >= packet.size())
				continue;

			uint16_t azimuth = packet[blockOffset] | (packet[blockOffset + 1] << 8);
			float azimuthRadians = (azimuth / 100.0f) * (M_PI / 180.0f);

			// Process the 32 laser measurements in this block
			for (int laser = 0; laser < 32; laser++) {
				int laserOffset = blockOffset + 4 + laser * 3;
				if (laserOffset + 2 >= packet.size())
					continue;

				uint16_t distance = packet[laserOffset] | (packet[laserOffset + 1] << 8);
				uint8_t intensity = packet[laserOffset + 2];

				if (distance == 0)
					continue;  // Invalid measurement

				// Convert to meters
				float distanceMeters = distance * 0.002f;

				// Get vertical angle for this laser
				float verticalAngle = config.velodyneVerticalAngles[laser % 64];
				float verticalRadians = verticalAngle * (M_PI / 180.0f);

				// Calculate x, y, z coordinates
				PointType point;
				point.x = distanceMeters * cos(verticalRadians) * sin(azimuthRadians);
				point.y = distanceMeters * cos(verticalRadians) * cos(azimuthRadians);
				point.z = distanceMeters * sin(verticalRadians);
				point.intensity = intensity;
				point.ring = laser % 64;

				cloud->push_back(point);
			}
		}
	}

	return cloud;
}

void OfflineProcessor::processAllData() {
	if (lidarFrames.empty()) {
		LOG_WARNING("No LiDAR frames to process. Load a bag file first.");
		return;
	}

	LOG_INFO("Processing " + std::to_string(lidarFrames.size()) + " LiDAR frames...");

	// Initialize accumulated cloud (will be periodically saved and cleared)
	accumulatedCloud.reset(new pcl::PointCloud<PointType>());

	// Always start from the first frame
	int startFrame = 0;

	// Process each frame
	auto startTime = std::chrono::steady_clock::now();

	LOG_INFO("Using memory-efficient processing with batch size: " + std::to_string(config.framesBatchSize)
			 + " frames");
	LOG_INFO("Accumulated cloud will be saved to: " + config.outputDirectory + "/"
			 + config.accumulatedCloudPath);

	for (int i = startFrame; i < static_cast<int>(lidarFrames.size()); i++) {
		try {
			processSingleFrame(i);

			// Report progress periodically
			if (i % 10 == 0 || i == static_cast<int>(lidarFrames.size()) - 1) {
				reportProgress(i, lidarFrames.size());

				// Generate a progress visualization if needed
				if (i % 50 == 0 && i > 0) {
					std::string progressPCD
						= config.outputDirectory + "/progress_" + std::to_string(i) + ".pcd";
					// visualizeProgressPCD(progressPCD);
				}
			}
		} catch (const std::exception& e) {
			LOG_ERROR(std::string("Error processing frame ") + std::to_string(i) + ": " + e.what());
			LOG_ERROR("Attempting to continue with next frame...");
		}
	}

	// Final optimization
	// LOG_INFO("Performing final map optimization...");
	// mapOptimization->optimizeMap();

	// Final progress report
	auto endTime = std::chrono::steady_clock::now();
	auto totalTime = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
	LOG_INFO("Processing complete! Total time: " + std::to_string(totalTime) + " seconds");
	LOG_INFO("Final accumulated point cloud saved to: " + config.outputDirectory + "/"
			 + config.accumulatedCloudPath);
}

void OfflineProcessor::processSingleFrame(int frameIdx) {
	auto& frame = lidarFrames[frameIdx];

	// Get relevant IMU data for this frame
	// auto frameImuData = getImuDataForTimespan(frame.timestamp, 0.1);
	double frameTimespan = 0.1;	 // seconds fix
	double startTime = frame.timestamp;
	double endTime = frame.timestamp + frameTimespan;
	double interval = 0.01;	 // fix
	// Get initial pose guess from odometry
	std::vector<OdomData> frameOdomData = getInterpolatedOdomDataRange(startTime, endTime, interval);

	OdomData initialOdomPose;
	if (!frameOdomData.empty()) {
		initialOdomPose = frameOdomData.front();
	} else {
		// Create default identity pose
		initialOdomPose.timestamp = frame.timestamp;
		initialOdomPose.transform = Eigen::Matrix4f::Identity();
		initialOdomPose.linear_velocity = Eigen::Vector3f::Zero();
		initialOdomPose.angular_velocity = Eigen::Vector3f::Zero();
		LOG_WARNING("No odometry data available for frame " + std::to_string(frameIdx));
	}

	// Process through the pipeline
	pcl::PointCloud<PointType>::Ptr deskewedCloud(new pcl::PointCloud<PointType>);

	// Image projection (deskewing)

	auto cloudInfo = imageProjection->processPointCloudWithOdom(frame.cloud, this, deskewedCloud);
	LOG_INFO("Deskewed cloud with " + std::to_string(deskewedCloud->size()) + " points");
	// Add to accumulated cloud
	if (!accumulatedCloud) {
		accumulatedCloud.reset(new pcl::PointCloud<PointType>());
	}

	*accumulatedCloud += *deskewedCloud;

	// Check if we need to save and clear memory
	bool isLastFrame = (frameIdx == static_cast<int>(lidarFrames.size()) - 1);
	bool isBatchBoundary = (frameIdx % config.framesBatchSize == config.framesBatchSize - 1);
	if (isLastFrame || isBatchBoundary) {
		// No downsampling anymore

		// Save to file
		std::string outputPath = config.outputDirectory + "/" + config.accumulatedCloudPath;
		LOG_INFO("Saving accumulated cloud to: " + outputPath);
		if (config.appendToExistingPCD && frameIdx > config.framesBatchSize) {
			// Append to existing file by loading, combining, and saving
			appendToExistingPointCloud(outputPath, accumulatedCloud);
			LOG_INFO("Appended to existing PCD file: " + outputPath);
		} else {
			// Save directly (overwrite or first batch)
			savePointCloud(outputPath, accumulatedCloud);
			LOG_INFO("Saved new PCD file: " + outputPath);
		}
		// Clear memory
		LOG_INFO("Saved batch ending at frame " + std::to_string(frameIdx) + " with "
				 + std::to_string(accumulatedCloud->size()) + " points. Clearing memory.");
		accumulatedCloud->clear();
	}
	// // Feature extraction
	// pcl::PointCloud<PointType>::Ptr cornerCloud(new pcl::PointCloud<PointType>);
	// pcl::PointCloud<PointType>::Ptr surfCloud(new pcl::PointCloud<PointType>);
	// featureExtraction->extractFeatures(deskewedCloud, cloudInfo, cornerCloud, surfCloud);
	// // Map optimization
	// mapOptimization->addKeyFrame(cornerCloud, surfCloud, frame.timestamp, initialPose);
}

OdomData OfflineProcessor::getInterpolatedOdomData(double timestamp) {
	OdomData result;
	result.timestamp = timestamp;
	if (odomData.empty()) {
		result.transform = Eigen::Matrix4f::Identity();
		result.linear_velocity = Eigen::Vector3f::Zero();
		result.angular_velocity = Eigen::Vector3f::Zero();
		return result;
	}
	// If timestamp is before the first odometry message, use the first one
	if (timestamp <= odomData.front().timestamp) {
		return odomData.front();
	}
	// If timestamp is after the last odometry message, use the last one
	if (timestamp >= odomData.back().timestamp) {
		return odomData.back();
	}
	// Find the two odometry messages that bracket the target timestamp
	size_t beforeIdx = 0;
	size_t afterIdx = 0;
	for (size_t i = 0; i < odomData.size() - 1; i++) {
		if (odomData[i].timestamp <= timestamp && timestamp <= odomData[i + 1].timestamp) {
			beforeIdx = i;
			afterIdx = i + 1;
			break;
		}
	}

	// Calculate interpolation ratio
	double beforeTime = odomData[beforeIdx].timestamp;
	double afterTime = odomData[afterIdx].timestamp;
	double ratio = (timestamp - beforeTime) / (afterTime - beforeTime);
	// Ensure ratio is between 0 and 1

	ratio = std::max(0.0, std::min(1.0, ratio));
	// Linear interpolation for translation

	Eigen::Vector3f translation = (1.0 - ratio) * odomData[beforeIdx].transform.block<3, 1>(0, 3)
								  + ratio * odomData[afterIdx].transform.block<3, 1>(0, 3);
	// SLERP for rotation with improved handling

	Eigen::Matrix3f rotBefore = odomData[beforeIdx].transform.block<3, 3>(0, 0);
	Eigen::Matrix3f rotAfter = odomData[afterIdx].transform.block<3, 3>(0, 0);
	// Convert to quaternions for interpolation

	Eigen::Quaternionf qBefore(rotBefore);
	Eigen::Quaternionf qAfter(rotAfter);
	// Normalize quaternions to ensure valid rotation

	qBefore.normalize();
	qAfter.normalize();
	// Check for shortest path interpolation
	// If dot product is negative, one quaternion should be negated

	if (qBefore.dot(qAfter) < 0.0f) {
		qAfter.coeffs() = -qAfter.coeffs();
	}
	// Perform spherical linear interpolation

	Eigen::Quaternionf qInterp = qBefore.slerp(ratio, qAfter);
	// Normalize the result to ensure a valid rotation

	qInterp.normalize();
	// Linear interpolation for velocities

	Eigen::Vector3f linearVel
		= (1.0 - ratio) * odomData[beforeIdx].linear_velocity + ratio * odomData[afterIdx].linear_velocity;
	// For angular velocity, we also need careful interpolation

	Eigen::Vector3f angularVel
		= (1.0 - ratio) * odomData[beforeIdx].angular_velocity + ratio * odomData[afterIdx].angular_velocity;
	// Combine into result

	result.transform = Eigen::Matrix4f::Identity();
	result.transform.block<3, 3>(0, 0) = qInterp.toRotationMatrix();
	result.transform.block<3, 1>(0, 3) = translation;
	result.linear_velocity = linearVel;
	result.angular_velocity = angularVel;

	// Add logging to debug interpolation issues
	if (ratio > 0.01 && ratio < 0.99) {	 // Only log when actually interpolating
		Eigen::Vector3f beforeEuler = extractEulerAngles(rotBefore);
		Eigen::Vector3f afterEuler = extractEulerAngles(rotAfter);
		Eigen::Vector3f interpEuler = extractEulerAngles(result.transform.block<3, 3>(0, 0));
		// std::stringstream ss;
		// ss << "Interpolation at ratio " << ratio << ":\n"
		//    << "  Before RPY: " << beforeEuler.transpose() * 180 / M_PI << " deg\n"
		//    << "  After RPY: " << afterEuler.transpose() * 180 / M_PI << " deg\n"
		//    << "  Interp RPY: " << interpEuler.transpose() * 180 / M_PI << " deg";
		// LOG_INFO(ss.str());
	}
	return result;
}
// Helper function to extract Euler angles consistently
Eigen::Vector3f OfflineProcessor::extractEulerAngles(const Eigen::Matrix3f& rotation) {
	// Extract roll, pitch, yaw using a consistent convention
	// This uses the ZYX convention (yaw, pitch, roll)
	Eigen::Vector3f euler;

	// Handle the special case of gimbal lock
	if (std::abs(rotation(2, 0)) > 0.99999f) {
		// Gimbal lock case
		euler[0] = 0;  // roll

		if (rotation(2, 0) > 0) {
			euler[1] = -M_PI / 2;									 // pitch = -90 degrees
			euler[2] = -std::atan2(rotation(0, 1), rotation(1, 1));	 // yaw
		} else {
			euler[1] = M_PI / 2;									// pitch = 90 degrees
			euler[2] = std::atan2(rotation(0, 1), rotation(1, 1));	// yaw
		}
	} else {
		// Normal case (no gimbal lock)
		euler[0] = std::atan2(rotation(2, 1), rotation(2, 2));	// roll
		euler[1] = std::asin(-rotation(2, 0));					// pitch
		euler[2] = std::atan2(rotation(1, 0), rotation(0, 0));	// yaw
	}

	return euler;
}
std::vector<OdomData> OfflineProcessor::getInterpolatedOdomDataRange(double startTime,
																	 double endTime,
																	 double interval) {
	std::vector<OdomData> result;

	if (interval <= 0.0) {
		return result;	// Invalid interval
	}

	// Calculate how many interpolated points we need
	int numPoints = static_cast<int>((endTime - startTime) / interval) + 1;

	// Generate interpolated points
	for (int i = 0; i < numPoints; i++) {
		double timestamp = startTime + i * interval;
		if (timestamp > endTime)
			break;

		OdomData interpolated = getInterpolatedOdomData(timestamp);
		result.push_back(interpolated);
	}

	return result;
}

void OfflineProcessor::reportProgress(int frameIdx, int totalFrames) {
	auto now = std::chrono::steady_clock::now();
	static auto lastReportTime = now;
	static int lastFrameIdx = 0;

	float percentage = (float)(frameIdx + 1) / totalFrames * 100.0f;

	// Calculate frames per second
	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastReportTime).count();
	if (elapsed > 0) {
		float fps = static_cast<float>(frameIdx - lastFrameIdx) / elapsed;
		std::stringstream ss;
		ss << "Progress: " << percentage << "% (" << (frameIdx + 1) << "/" << totalFrames << ") - " << fps
		   << " frames/sec";
		LOG_INFO(ss.str());

		lastReportTime = now;
		lastFrameIdx = frameIdx;
	} else {
		std::stringstream ss;
		ss << "Progress: " << percentage << "% (" << (frameIdx + 1) << "/" << totalFrames << ")";
		LOG_INFO(ss.str());
	}
}

// void OfflineProcessor::visualizeProgressPCD(const std::string& filename) {
// 	// Get current map and save to PCD for visualization
// 	pcl::PointCloud<PointType>::Ptr currentMap = mapOptimization->getGlobalMap();
// 	if (currentMap && !currentMap->empty()) {
// 		pcl::io::savePCDFileBinary(filename, *currentMap);
// 		LOG_INFO("Saved progress visualization to " + filename);
// 	}
// }

// Method implementations for saving point clouds
bool OfflineProcessor::savePointCloud(const std::string& filePath, pcl::PointCloud<PointType>::Ptr cloud) {
	if (!cloud || cloud->empty()) {
		LOG_WARNING("Empty point cloud, not saving: " + filePath);
		return false;
	}

	try {
		auto xyziCloud = convertToXYZI(cloud);
		pcl::io::savePCDFileBinary(filePath, *xyziCloud);
		return true;
	} catch (const std::exception& e) {
		LOG_ERROR("Error saving point cloud: " + std::string(e.what()));
		return false;
	}
}

void OfflineProcessor::appendToExistingPointCloud(const std::string& filePath,
												  pcl::PointCloud<PointType>::Ptr newCloud) {
	if (!newCloud || newCloud->empty()) {
		LOG_WARNING("No new points to append");
		return;
	}

	// Check if the existing file exists
	if (!std::filesystem::exists(filePath)) {
		// If not, just save the new cloud directly
		savePointCloud(filePath, newCloud);
		return;
	}

	try {
		// Load the existing cloud
		pcl::PointCloud<PointType>::Ptr existingCloud(new pcl::PointCloud<PointType>());
		if (pcl::io::loadPCDFile<PointType>(filePath, *existingCloud) == -1) {
			LOG_WARNING("Could not load existing point cloud: " + filePath + ". Creating new file.");
			savePointCloud(filePath, newCloud);
			return;
		}
		LOG_INFO("Loaded existing cloud with " + std::to_string(existingCloud->size()) + " points. Appending "
				 + std::to_string(newCloud->size()) + " new points.");
		// Combine clouds
		*existingCloud += *newCloud;
		// Save combined cloud without downsampling
		savePointCloud(filePath, existingCloud);
	} catch (const std::exception& e) {
		LOG_ERROR("Error appending to point cloud: " + std::string(e.what()));
		// Fallback: save just the new data
		std::string backupPath = filePath + ".backup";
		savePointCloud(backupPath, newCloud);
	}
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OfflineProcessor::convertToXYZI(
	pcl::PointCloud<PointType>::Ptr inputCloud) {
	auto xyziCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	xyziCloud->reserve(inputCloud->size());

	for (const auto& pt : *inputCloud) {
		pcl::PointXYZI out;
		out.x = pt.x;
		out.y = pt.y;
		out.z = pt.z;
		out.intensity = pt.intensity;
		xyziCloud->push_back(out);
	}

	return xyziCloud;
}

bool OfflineProcessor::saveResults() {
	LOG_INFO("Saving final results...");

	std::string mapFile = config.outputDirectory + "/final_map.pcd";
	if (!mapOptimization->saveMap(mapFile)) {
		LOG_ERROR("Failed to save map to " + mapFile);
		return false;
	}

	// Save trajectory to file
	std::string trajectoryFile = config.outputDirectory + "/trajectory.txt";
	std::ofstream trajFile(trajectoryFile);
	if (trajFile.is_open()) {
		auto trajectory = mapOptimization->getTrajectory();
		trajFile << "# timestamp x y z roll pitch yaw" << std::endl;
		for (const auto& pose : trajectory) {
			trajFile << pose.time << " " << pose.x << " " << pose.y << " " << pose.z << " " << pose.roll
					 << " " << pose.pitch << " " << pose.yaw << std::endl;
		}
		trajFile.close();
	} else {
		LOG_ERROR("Failed to save trajectory to " + trajectoryFile);
	}

	LOG_INFO("Results saved to " + config.outputDirectory);
	return true;
}

std::vector<OfflineProcessor::ImuData> OfflineProcessor::getImuDataForTimespan(double timestamp,
																			   double timeRadius) {
	std::vector<ImuData> result;

	// Find IMU data within the specified time range
	for (const auto& imu : imuData) {
		if (std::abs(imu.timestamp - timestamp) <= timeRadius) {
			result.push_back(imu);
		}
	}

	return result;
}

void OfflineProcessor::visualizeOdomTopics(const std::string& outputDir) {
	LOG_INFO("Visualizing odometry coordinate systems...");

	// Initialize point cloud to hold all axes using a standard PCL point type
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr axesCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Sample time points to visualize
	const double timeStep = 0.5;  // 0.5 seconds between samples
	const double startTime = getDataStartTime();
	const double endTime = getDataEndTime();

	LOG_INFO("Generating axes from time " + std::to_string(startTime) + " to " + std::to_string(endTime)
			 + " with step " + std::to_string(timeStep));

	// For each time step
	for (double time = startTime; time <= endTime; time += timeStep) {
		// Get odometry at this time
		OdomData odom = getInterpolatedOdomData(time);

		// Extract position and rotation from odometry
		Eigen::Vector3f position = odom.transform.block<3, 1>(0, 3);
		Eigen::Matrix3f rotation = odom.transform.block<3, 3>(0, 0);

		// Get axes directions
		Eigen::Vector3f x_axis = rotation.col(0);
		Eigen::Vector3f y_axis = rotation.col(1);
		Eigen::Vector3f z_axis = rotation.col(2);

		// Scale factor for axes
		const float scale = 2.0;

		// Add origin point
		pcl::PointXYZRGB origin;
		origin.x = position.x();
		origin.y = position.y();
		origin.z = position.z();
		// White color for origin
		origin.r = 255;
		origin.g = 255;
		origin.b = 255;
		axesCloud->push_back(origin);

		// Helper function to draw a line with multiple points
		auto drawLine
			= [&](const Eigen::Vector3f& start, const Eigen::Vector3f& end, uint8_t r, uint8_t g, uint8_t b) {
				  const int numSegments = 10;
				  for (int i = 0; i <= numSegments; i++) {
					  float t = static_cast<float>(i) / numSegments;
					  Eigen::Vector3f pos = start + t * (end - start);

					  pcl::PointXYZRGB pt;
					  pt.x = pos.x();
					  pt.y = pos.y();
					  pt.z = pos.z();
					  pt.r = r;
					  pt.g = g;
					  pt.b = b;
					  axesCloud->push_back(pt);
				  }
			  };

		// Draw X axis (red)
		Eigen::Vector3f x_end = position + x_axis * scale;
		drawLine(position, x_end, 255, 0, 0);

		// Draw Y axis (green)
		Eigen::Vector3f y_end = position + y_axis * scale;
		drawLine(position, y_end, 0, 255, 0);

		// Draw Z axis (blue)
		Eigen::Vector3f z_end = position + z_axis * scale;
		drawLine(position, z_end, 0, 0, 255);

		// Add arrow heads
		const float headSize = scale * 0.2f;

		// Add arrow heads with the same approach as before
		// X, Y, Z axis arrow heads...
	}

	// Check if we generated any points
	if (axesCloud->empty()) {
		LOG_ERROR("No axes points were generated. Check your odometry data.");
		return;
	}

	// Save the axes cloud
	std::string outputFile = outputDir + "/odometry_axes_visualization.pcd";
	pcl::io::savePCDFile(outputFile, *axesCloud);
	LOG_INFO("Saved odometry visualization to " + outputFile + " with " + std::to_string(axesCloud->size())
			 + " points");

	// Provide instructions for viewing the file instead of launching the visualizer
	LOG_INFO("To view the visualization, run one of these commands:");
	LOG_INFO("  pcl_viewer " + outputFile);
	LOG_INFO("  or");
	LOG_INFO("  cloudcompare.CloudCompare " + outputFile);
}
double OfflineProcessor::getDataStartTime() const {
	if (odomData.empty()) {
		return 0.0;
	}
	return odomData.front().timestamp;
}

double OfflineProcessor::getDataEndTime() const {
	if (odomData.empty()) {
		return 0.0;
	}
	return odomData.back().timestamp;
}