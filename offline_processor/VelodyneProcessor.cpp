// VelodyneProcessor.cpp
#include "VelodyneProcessor.hpp"
#include <arpa/inet.h>
#include <netinet/ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <cstring>

VelodyneProcessor::VelodyneProcessor(const std::string& pcapFile,
									 const std::string& deviceIp,
									 uint16_t port,
									 bool readOnce,
									 bool readFast)
	: filename_(pcapFile),
	  deviceIp_(deviceIp),
	  port_(port),
	  readOnce_(readOnce),
	  readFast_(readFast),
	  pcap_(nullptr),
	  empty_(true) {
	initializeDefaultCalibration();
}

VelodyneProcessor::~VelodyneProcessor() {
	if (pcap_) {
		pcap_close(pcap_);
	}
}

bool VelodyneProcessor::open() {
	// Open the PCAP file
	char errbuf[PCAP_ERRBUF_SIZE];
	pcap_ = pcap_open_offline(filename_.c_str(), errbuf);

	if (pcap_ == nullptr) {
		std::cerr << "Error opening PCAP file: " << errbuf << std::endl;
		return false;
	}

	// Set up packet filter
	std::stringstream filter;
	if (!deviceIp_.empty()) {
		filter << "src host " << deviceIp_ << " && ";
	}
	filter << "udp port " << port_;

	if (pcap_compile(pcap_, &packetFilter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN) < 0) {
		std::cerr << "Failed to compile PCAP filter: " << pcap_geterr(pcap_) << std::endl;
		return false;
	}

	std::cout << "Opened Velodyne PCAP file: " << filename_ << std::endl;
	std::cout << "Filter: " << filter.str() << std::endl;

	return true;
}

bool VelodyneProcessor::getPacket(std::vector<uint8_t>& packet, double& timestamp) {
	struct pcap_pkthdr* header;
	const u_char* pktData;

	while (true) {
		int res = pcap_next_ex(pcap_, &header, &pktData);

		if (res >= 0) {
			// Skip packets not matching our filter
			if (0 == pcap_offline_filter(&packetFilter_, header, pktData)) {
				continue;
			}
			// Copy packet data (skip Ethernet + IP + UDP headers)
			const size_t packetSize = 1206;	 // Velodyne packet size
			packet.resize(packetSize);

			// Check if we have enough data
			if (header->caplen < 42 + packetSize) {
				std::cerr << "Truncated packet: " << header->caplen << " bytes" << std::endl;
				continue;
			}

			std::memcpy(packet.data(), pktData + 42, packetSize);

			// Extract timestamp from Velodyne packet - use GPS time
			uint32_t gpsTimestamp = 0;
			if (packet.size() >= 1204) {
				gpsTimestamp
					= packet[1200] | (packet[1201] << 8) | (packet[1202] << 16) | (packet[1203] << 24);
			}

			// Convert GPS timestamp to Unix time
			timestamp
				= convertGpsTimestampToUnixTime(gpsTimestamp, header->ts.tv_sec + header->ts.tv_usec * 1e-6);

			empty_ = false;
			return true;
		}

		// End of file
		if (empty_) {
			std::cerr << "Error reading Velodyne packet: " << pcap_geterr(pcap_) << std::endl;
			return false;
		}

		if (readOnce_) {
			std::cout << "End of file reached - done reading." << std::endl;
			return false;
		}

		// Reopen the file
		std::cout << "Replaying Velodyne PCAP file" << std::endl;
		pcap_close(pcap_);
		char errbuf[PCAP_ERRBUF_SIZE];
		pcap_ = pcap_open_offline(filename_.c_str(), errbuf);
		empty_ = true;
	}
}

std::shared_ptr<pcl::PointCloud<VelodynePoint>> VelodyneProcessor::convertPacketToPoints(
	const std::vector<uint8_t>& packet,
	double timestamp) {
	auto cloud = std::make_shared<pcl::PointCloud<VelodynePoint>>();

	// Check packet size
	if (packet.size() < 1206) {
		std::cerr << "Invalid packet size: " << packet.size() << " bytes" << std::endl;
		return cloud;
	}

	// Distance resolution from calibration (default to 0.004 meters per count for VLP-32C)
	// This matches the value in the YAML calibration file
	float distanceResolution = 0.004f;

	// Check packet factory bytes (the last two bytes should be 0x37 0x22 for VLP-32C)
	uint16_t factoryByte = (packet[1204] << 8) | packet[1205];
	bool isDualReturn = false;

	// Identify return mode by factory bytes
	if (factoryByte == 0x3722) {
		isDualReturn = false;  // Single return mode
	} else if (factoryByte == 0x3723) {
		isDualReturn = true;  // Dual return mode
	}

	// Process each block in the packet
	// VLP-32C has 12 blocks per packet
	for (int block = 0; block < 12; block++) {
		// Calculate block offset (each block is 100 bytes)
		int blockOffset = block * 100;
		// Extract block flag using little-endian interpretation
		uint16_t flag = (packet[blockOffset] | (packet[blockOffset + 1] << 8));

		// Check for valid flags
		bool isValidBlock = (flag == 0xEEFF || flag == 0xDDFF);

		if (!isValidBlock) {
			std::cerr << "Invalid block flag: " << std::hex << flag << std::endl;
			continue;
		}

		// Extract azimuth (rotation angle)
		uint16_t azimuth = (packet[blockOffset + 2] << 8) | packet[blockOffset + 3];
		float azimuthRadians = (azimuth / 100.0f) * (M_PI / 180.0f);

		// Process each firing in this block (32 lasers for VLP-32C)
		for (int laser = 0; laser < 32; laser++) {
			int pointOffset = blockOffset + 4 + laser * 3;
			// Extract distance and intensity
			uint16_t distance = (packet[pointOffset] << 8) | packet[pointOffset + 1];
			uint8_t intensity = packet[pointOffset + 2];

			// Skip invalid points
			if (distance == 0) {
				continue;
			}

			// Convert distance to meters using the calibration resolution
			float distanceMeters = distance * distanceResolution;

			// Get vertical angle for this laser
			float verticalAngle = calibration_.vertical_angles[laser];
			float verticalRadians = verticalAngle * (M_PI / 180.0f);

			// Apply horizontal (rotation) correction from calibration
			float rotCorrection = calibration_.horizontal_offsets[laser] * (M_PI / 180.0f);
			float correctedAzimuthRadians = azimuthRadians + rotCorrection;

			// Apply distance correction
			distanceMeters += calibration_.distance_corrections[laser];

			// Apply vertical offset correction
			float vertOffsetCorrection = calibration_.vertical_corrections[laser] * (M_PI / 180.0f);
			float correctedVerticalRadians = verticalRadians + vertOffsetCorrection;

			// Calculate 3D coordinates
			VelodynePoint point;
			point.x = distanceMeters * cos(correctedVerticalRadians) * sin(correctedAzimuthRadians);
			point.y = distanceMeters * cos(correctedVerticalRadians) * cos(correctedAzimuthRadians);
			point.z = distanceMeters * sin(correctedVerticalRadians);
			point.intensity = intensity;
			point.ring = laser;

			// Calculate precise firing time
			point.time = calculateFiringTime(azimuth, block, laser);

			// Store raw values for reference
			point.distance = distance;
			point.azimuth = azimuth;
			point.return_type = isDualReturn ? (flag == 0xEEFF ? 1 : 2) : 0;
			point.dual_return = isDualReturn;

			cloud->push_back(point);
		}
	}

	return cloud;
}

bool VelodyneProcessor::isNewScan(const std::vector<uint8_t>& packet, uint16_t& lastAzimuth) {
	if (packet.size() < 100) {
		return false;
	}

	// Extract azimuth from the first block
	uint16_t firstAzimuth = (packet[2] << 8) | packet[3];

	// If the new azimuth is less than the last one by a significant amount,
	// we've wrapped around (crossed from ~359° to ~0°)
	if (lastAzimuth > firstAzimuth && (lastAzimuth - firstAzimuth) > 18000) {
		lastAzimuth = firstAzimuth;
		return true;
	}

	lastAzimuth = firstAzimuth;
	return false;
}

void VelodyneProcessor::initializeDefaultCalibration() {
	// Initialize VLP-32C vertical angles
	// These values come from the Velodyne documentation
	float vlp32c_vertical_angles[32]
		= {-25.0f,	-1.0f,	 -1.667f, -15.639f, -11.31f, 0.0f,	  -0.667f, -8.843f, -7.254f, 0.333f,  -0.333f,
		   -6.148f, -5.333f, 1.333f,  0.667f,	-4.0f,	 -4.667f, 1.667f,  1.0f,	-3.667f, -3.333f, 3.333f,
		   2.333f,	-2.667f, -3.0f,	  7.0f,		4.667f,	 -2.333f, -2.0f,   15.0f,	10.333f, -1.333f};

	// Copy the values to our calibration structure
	for (int i = 0; i < 32; i++) {
		calibration_.vertical_angles[i] = vlp32c_vertical_angles[i];

		// In a real implementation, we would load these from a calibration file
		calibration_.vertical_corrections[i] = 0.0f;  // Default: no correction
		calibration_.horizontal_offsets[i] = 0.0f;	  // Default: no correction
		calibration_.distance_corrections[i] = 0.0f;  // Default: no correction
	}
}

double VelodyneProcessor::convertGpsTimestampToUnixTime(uint32_t gpsTimestamp, double pcapTime) {
	// If no valid GPS timestamp (occurs with simulated data), use PCAP time
	if (gpsTimestamp == 0) {
		return pcapTime;
	}

	// The GPS timestamp is microseconds since the top of the hour
	double secondsSinceHour = gpsTimestamp / 1000000.0;

	// Get the top of the hour from the PCAP timestamp
	time_t pcapTimeSec = static_cast<time_t>(pcapTime);
	struct tm* timeinfo = gmtime(&pcapTimeSec);

	// Reset minutes and seconds to get the top of the hour
	timeinfo->tm_min = 0;
	timeinfo->tm_sec = 0;

	// Convert back to Unix timestamp
	time_t topOfHour = mktime(timeinfo);

	// Add the seconds since the top of the hour
	return topOfHour + secondsSinceHour;
}

float VelodyneProcessor::calculateFiringTime(uint16_t azimuth, int blockIndex, int laserIndex) {
	// Calculate the precise firing time based on azimuth, block, and laser

	// First, convert azimuth to a fraction of a full rotation
	float rotationFraction = azimuth / 36000.0f;  // Full rotation is 36000 centidegrees

	// Calculate time for a full rotation
	float rotationPeriod = 1.0f / rotation_rate_;  // seconds per rotation

	// Time offset within the rotation
	float baseTime = rotationFraction * rotationPeriod;

	// Fine timing adjustment based on firing sequence
	// VLP-32C fires all 32 lasers simultaneously (unlike older models)
	// So time offset is mainly due to the block position
	float blockOffset = static_cast<float>(blockIndex) / 12.0f * rotationPeriod;

	// The laser index doesn't contribute to timing in VLP-32C
	// but we include it for completeness
	float laserOffset = 0.0f;  // No additional offset for laser index in VLP-32C

	return baseTime + blockOffset + laserOffset;
}

void VelodyneProcessor::applyCalibrationCorrections(float& distance,
													float& azimuthRadians,
													float& verticalRadians,
													uint8_t laserIndex,
													uint8_t& intensity) {
	// Apply various calibration corrections
	// In a real implementation, these would be loaded from a calibration file

	// Apply distance correction
	distance += calibration_.distance_corrections[laserIndex];

	// Apply vertical angle correction
	verticalRadians += calibration_.vertical_corrections[laserIndex] * (M_PI / 180.0f);

	// Apply horizontal angle correction
	azimuthRadians += calibration_.horizontal_offsets[laserIndex] * (M_PI / 180.0f);

	// More complex corrections can be applied based on the calibration data
}

bool VelodyneProcessor::loadCalibration(const std::string& calibrationFile) {
	try {
		// Load YAML file
		YAML::Node config = YAML::LoadFile(calibrationFile);

		if (!config["lasers"] || !config["lasers"].IsSequence()) {
			std::cerr << "Error: Invalid calibration file format - lasers sequence missing" << std::endl;
			return false;
		}

		// Check if the number of lasers matches
		if (config["lasers"].size() != 32) {
			std::cerr << "Warning: Expected 32 lasers, found " << config["lasers"].size() << std::endl;
		}

		// Reset calibration data
		calibration_.hasCalibration = false;

		// Process each laser's calibration data
		for (size_t i = 0; i < config["lasers"].size() && i < 32; i++) {
			const YAML::Node& laser = config["lasers"][i];

			// Get laser id
			int laser_id = laser["laser_id"].as<int>();

			// Check if laser_id is valid
			if (laser_id < 0 || laser_id >= 32) {
				std::cerr << "Warning: Invalid laser_id: " << laser_id << std::endl;
				continue;
			}

			// Load calibration values
			// Note: We need to convert from radians to degrees for some values

			// Vertical correction (vert_correction) - in radians, convert to degrees for our use
			if (laser["vert_correction"]) {
				float vert_correction_rad = laser["vert_correction"].as<float>();
				calibration_.vertical_angles[laser_id] = vert_correction_rad * 180.0f / M_PI;
			}

			// Horizontal/rotation correction (rot_correction) - in radians
			if (laser["rot_correction"]) {
				float rot_correction_rad = laser["rot_correction"].as<float>();
				calibration_.horizontal_offsets[laser_id] = rot_correction_rad * 180.0f / M_PI;
			}

			// Distance correction - in meters
			if (laser["dist_correction"]) {
				calibration_.distance_corrections[laser_id] = laser["dist_correction"].as<float>();
			}

			// Vertical offset correction
			if (laser["vert_offset_correction"]) {
				calibration_.vertical_corrections[laser_id] = laser["vert_offset_correction"].as<float>();
			}
		}

		// Also check for distance resolution
		if (config["distance_resolution"]) {
			float distance_resolution = config["distance_resolution"].as<float>();
			// Typically this should be 0.004, or 4mm for VLP-32C (different from the 2mm we assumed earlier)
			std::cout << "Distance resolution: " << distance_resolution << " meters" << std::endl;
		}

		// Check for number of lasers
		if (config["num_lasers"]) {
			int num_lasers = config["num_lasers"].as<int>();
			if (num_lasers != 32) {
				std::cerr << "Warning: Calibration file reports " << num_lasers
						  << " lasers, but expected 32 for VLP-32C" << std::endl;
			}
		}

		calibration_.hasCalibration = true;
		std::cout << "Successfully loaded calibration from: " << calibrationFile << std::endl;

		// Debug: print a few calibration values
		std::cout << "Sample calibration values:" << std::endl;
		for (int i = 0; i < 3; i++) {
			std::cout << "  Laser " << i << ": Vertical angle: " << calibration_.vertical_angles[i]
					  << "°, Horizontal offset: " << calibration_.horizontal_offsets[i]
					  << "°, Distance correction: " << calibration_.distance_corrections[i] << "m"
					  << std::endl;
		}

		return true;
	} catch (const std::exception& e) {
		std::cerr << "Error loading calibration file: " << e.what() << std::endl;
		return false;
	}
}