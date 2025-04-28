// VelodyneProcessor.hpp
#pragma once

#include <pcap.h>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>	// Add dependency on yaml-cpp

// Define a more complete point type with all Velodyne data
struct VelodynePoint {
	float x, y, z;		  // Cartesian coordinates
	float intensity;	  // Intensity value
	uint8_t ring;		  // Laser ring/id
	float time;			  // Time offset relative to scan start
	uint16_t distance;	  // Raw distance value
	uint16_t azimuth;	  // Raw azimuth value
	uint8_t return_type;  // Return type (strongest, last, etc.)
	bool dual_return;	  // Is part of dual return
};

// Structure for VLP-32C calibration data
struct VelodyneCalibration {
	std::array<float, 32> vertical_angles;		  // Vertical angles
	std::array<float, 32> vertical_corrections;	  // Vertical corrections
	std::array<float, 32> horizontal_offsets;	  // Horizontal offsets
	std::array<float, 32> distance_corrections;	  // Distance corrections
	std::array<float, 32> intensity_corrections;  // Intensity corrections
	bool hasCalibration = false;				  // Flag if calibration is loaded
};

class VelodyneProcessor {
   public:
	VelodyneProcessor(const std::string& pcapFile,
					  const std::string& deviceIp = "",
					  uint16_t port = 2368,
					  bool readOnce = true,
					  bool readFast = false);

	~VelodyneProcessor();

	bool open();

	// Load calibration from Velodyne YAML file
	bool loadCalibration(const std::string& calibrationFile);

	// Get next packet from PCAP file
	bool getPacket(std::vector<uint8_t>& packet, double& timestamp);

	// Convert a packet to points with full decoding
	std::shared_ptr<pcl::PointCloud<VelodynePoint>> convertPacketToPoints(const std::vector<uint8_t>& packet,
																		  double timestamp);

	// Check if the packet indicates a new scan (azimuth wrap)
	bool isNewScan(const std::vector<uint8_t>& packet, uint16_t& lastAzimuth);

   private:
	std::string filename_;
	std::string deviceIp_;
	uint16_t port_;
	bool readOnce_;
	bool readFast_;
	pcap_t* pcap_;
	struct bpf_program packetFilter_;
	bool empty_;

	// VLP-32C calibration data
	VelodyneCalibration calibration_;

	// Rotation rate (in Hz)
	float rotation_rate_ = 10.0f;  // Default 10Hz for VLP-32C

	// Initialize default calibration for VLP-32C
	void initializeDefaultCalibration();

	// Convert GPS timestamp to Unix time
	double convertGpsTimestampToUnixTime(uint32_t gpsTimestamp, double pcapTime);

	// Calculate precise firing time for a laser within a scan
	float calculateFiringTime(uint16_t azimuth, int blockIndex, int laserIndex);

	// Apply calibration corrections to raw point data
	void applyCalibrationCorrections(float& distance,
									 float& azimuthRadians,
									 float& verticalRadians,
									 uint8_t laserIndex,
									 uint8_t& intensity);
};