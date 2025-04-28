#pragma once
#include <arpa/inet.h>		//
#include <netinet/ether.h>	// For Ethernet header
#include <netinet/ip.h>		// For IP header
#include <netinet/udp.h>	// For UDP header
#include <pcap.h>			// For PCAP functions
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "Logger.hpp"
#include "VelodyneProcessor.hpp"
#include "offline_ImageProjection.hpp"
#include "offline_utility.hpp"

// Forward declarations for classes we'll use via pointers only in this header
class OfflineFeatureExtraction;
class OfflineMapOptimization;
class OfflineImageProjection;

class OfflineProcessor {
   public:
	struct Config {
		std::string inputBagPath;
		std::string inputPcapPath;	// Add this for PCAP file support
		std::string outputDirectory;
		bool highQualityMode = true;
		float mapResolution = 0.1f;
		int nScan = 64;
		int horizonScan = 1800;
		float edgeThreshold = 0.1f;
		float surfThreshold = 0.1f;
		float mappingCornerLeafSize = 0.2f;
		float mappingSurfLeafSize = 0.4f;

		// PCAP-specific parameters
		std::string lidarModel = "VLP32C";	// Add this
		int lidarRPM = 600;					// Add this
		float minRange = 0.1f;				// Add this
		float maxRange = 100.0f;			// Add this
		uint16_t oxtsPort = 3000;			// Add this
		bool readFast = true;				// Add this

		// Memory management options
		bool useDownsampling = true;
		float downsamplingLeafSize = 0.05f;	 // 5cm voxel grid
		int framesBatchSize = 100;			 // Save and clear memory every N frames
		bool appendToExistingPCD = true;	 // Whether to append to existing PCD or create separate files
		std::string accumulatedCloudPath = "accumulated_cloud.pcd";	 // Path for the accumulated cloud
		int maxLidarMessages = 0;

		std::vector<float> velodyneVerticalAngles;
		// Constructor
		Config() {
			velodyneVerticalAngles = {-25.0f,  -1.0f,  -1.667f, -15.639f, -11.31f, 0.0f,   -0.667f, -8.843f,
									  -7.254f, 0.333f, -0.333f, -6.148f,  -5.333f, 1.333f, 0.667f,	-4.0f,
									  -4.667f, 1.667f, 1.0f,	-3.667f,  -3.333f, 3.333f, 2.333f,	-2.667f,
									  -3.0f,   7.0f,   4.667f,	-2.333f,  -2.0f,   15.0f,  10.333f, -1.333f};
			velodyneVerticalAngles.resize(64, 0.0f);  // Pad to 64 if needed
		}
	};

	struct LidarFrame {
		double timestamp;
		pcl::PointCloud<PointType>::Ptr cloud;
	};

	struct ImuData {
		double timestamp;
		Eigen::Vector3d angular_velocity;
		Eigen::Vector3d linear_acceleration;
		Eigen::Quaterniond orientation;
	};

	// Constructor and destructor
	OfflineProcessor(const Config& config);
	~OfflineProcessor();

	// Main processing methods
	bool loadBagFile();
	bool loadPcapFile();  // Add this function declaration
	void processAllData();
	void processSingleFrame(int frameIdx);
	bool saveResults();

	// Helper methods
	std::vector<ImuData> getImuDataForTimespan(double timestamp, double timeRadius);
	std::vector<OdomData> getInterpolatedOdomDataRange(double startTime, double endTime, double interval);
	OdomData getInterpolatedOdomData(double timestamp);
	void reportProgress(int frameIdx, int totalFrames);
	void visualizeProgressPCD(const std::string& filename);
	void visualizeOdomTopics(const std::string& outputDir);
	// Point cloud file operations
	bool savePointCloud(const std::string& filePath, pcl::PointCloud<PointType>::Ptr cloud);
	void appendToExistingPointCloud(const std::string& filePath, pcl::PointCloud<PointType>::Ptr newCloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr convertToXYZI(pcl::PointCloud<PointType>::Ptr inputCloud);
	double getDataStartTime() const;
	double getDataEndTime() const;

   private:
	// Configuration
	Config config;

	// Data storage
	std::vector<LidarFrame> lidarFrames;
	std::vector<ImuData> imuData;
	std::vector<OdomData> odomData;

	// Processing components
	std::unique_ptr<OfflineImageProjection> imageProjection;

	// Use these components only if needed - declare as nullptr in constructor until implemented
	std::unique_ptr<OfflineFeatureExtraction> featureExtraction;
	std::unique_ptr<OfflineMapOptimization> mapOptimization;
	pcl::PointCloud<PointType>::Ptr processVelodynePackets(const std::vector<std::vector<uint8_t>>& packets);
	Eigen::Vector3f extractEulerAngles(const Eigen::Matrix3f& rotation);

	// Accumulation storage
	pcl::PointCloud<PointType>::Ptr accumulatedCloud;
	std::shared_ptr<pcl::VoxelGrid<PointType>> downsamplingVoxelGrid;

	// Timing and progress
	std::chrono::time_point<std::chrono::steady_clock> processingStartTime;
};