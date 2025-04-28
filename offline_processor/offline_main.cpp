#include <filesystem>
#include <iostream>
#include <string>
#include "Logger.hpp"
#include "offline_Processor.hpp"

void printUsage(const char* programName) {
	std::cout << "Usage: " << programName << " <input_file> <output_directory> [options]" << std::endl;
	std::cout << "Input file can be a ROS2 bag directory or a PCAP file" << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout
		<< "  --file-type=<type>           Specify input file type (bag or pcap) (auto-detected by default)"
		<< std::endl;
	std::cout << "  --high-quality               Enable high quality mode (default)" << std::endl;
	std::cout << "  --standard-quality           Use standard quality mode" << std::endl;
	std::cout
		<< "  --max-lidar=<value>          Process only the first N LiDAR messages (default: process all)"
		<< std::endl;
	std::cout << "  --resolution=<value>         Set map resolution (default: 0.1)" << std::endl;
	std::cout << "  --n-scan=<value>             Set number of scan lines (default: 64)" << std::endl;
	std::cout << "  --horizon-scan=<value>       Set number of points per line (default: 1800)" << std::endl;
	std::cout << "  --edge-threshold=<value>     Set edge feature threshold (default: 0.1)" << std::endl;
	std::cout << "  --surf-threshold=<value>     Set surface feature threshold (default: 0.1)" << std::endl;
	std::cout << "  --convert-pcd-to-las         Direct conversion mode: convert input PCD to output LAS"
			  << std::endl;
	std::cout
		<< "  --log-level=<level>          Set log level (debug, info, warning, error, fatal) (default: info)"
		<< std::endl;
	std::cout << "  --log-file=<path>            Set log file path (default: processing.log)" << std::endl;

	// PCAP-specific options
	std::cout << "\nPCAP-specific options:" << std::endl;
	std::cout << "  --lidar-model=<model>        Set LiDAR model (VLP16, HDL32E, HDL64E) (default: HDL64E)"
			  << std::endl;
	std::cout << "  --lidar-rpm=<value>          Set LiDAR RPM (default: 600)" << std::endl;
	std::cout << "  --lidar-min-range=<value>    Set minimum LiDAR range in meters (default: 0.1)"
			  << std::endl;
	std::cout << "  --lidar-max-range=<value>    Set maximum LiDAR range in meters (default: 100.0)"
			  << std::endl;
	std::cout << "  --oxts-port=<value>          Set OXTS UDP port for IMU/GPS data (default: 3000)"
			  << std::endl;
	std::cout << "  --read-fast                  Process PCAP as fast as possible (default: true)"
			  << std::endl;
}

int main(int argc, char** argv) {
	// Check for direct conversion mode
	// Regular processing mode
	if (argc < 3) {
		printUsage(argv[0]);
		return 1;
	}

	std::string inputFilePath = argv[1];
	std::string outputDir = argv[2];

	// Default log settings
	std::string logFilePath = outputDir + "/processing.log";
	LogLevel logLevel = LogLevel::INFO;
	bool convertToLas = false;

	OfflineProcessor::Config config;
	config.outputDirectory = outputDir;
	config.highQualityMode = true;
	config.mapResolution = 0.1;

	// Auto-detect file type if not specified
	std::string fileExtension = std::filesystem::path(inputFilePath).extension().string();
	std::string fileType = "unknown";

	if (fileExtension == ".pcap") {
		fileType = "pcap";
		// Set default PCAP-specific settings
		config.inputPcapPath = inputFilePath;
		config.lidarModel = "VLP32C";
		config.lidarRPM = 600;
		config.minRange = 0.1;
		config.maxRange = 100.0;
		config.oxtsPort = 3000;
		config.readFast = true;
	} else {
		fileType = "bag";
		config.inputBagPath = inputFilePath;
	}

	for (int i = 3; i < argc; i++) {
		std::string arg = argv[i];

		if (arg.find("--file-type=") == 0) {
			fileType = arg.substr(12);
			if (fileType == "pcap") {
				config.inputPcapPath = inputFilePath;
			} else if (fileType == "bag") {
				config.inputBagPath = inputFilePath;
			} else {
				std::cerr << "Unknown file type: " << fileType << ". Use 'bag' or 'pcap'." << std::endl;
				return 1;
			}
		} else if (arg == "--high-quality") {
			config.highQualityMode = true;
		} else if (arg == "--standard-quality") {
			config.highQualityMode = false;
		} else if (arg.find("--max-lidar=") == 0) {
			config.maxLidarMessages = std::stoi(arg.substr(12));
		} else if (arg.find("--resolution=") == 0) {
			config.mapResolution = std::stof(arg.substr(13));
		} else if (arg.find("--n-scan=") == 0) {
			config.nScan = std::stoi(arg.substr(9));
		} else if (arg.find("--horizon-scan=") == 0) {
			config.horizonScan = std::stoi(arg.substr(15));
		} else if (arg.find("--edge-threshold=") == 0) {
			config.edgeThreshold = std::stof(arg.substr(17));
		} else if (arg.find("--surf-threshold=") == 0) {
			config.surfThreshold = std::stof(arg.substr(17));
		} else if (arg.find("--log-level=") == 0) {
			std::string levelStr = arg.substr(12);
			if (levelStr == "debug")
				logLevel = LogLevel::DEBUG;
			else if (levelStr == "info")
				logLevel = LogLevel::INFO;
			else if (levelStr == "warning")
				logLevel = LogLevel::WARNING;
			else if (levelStr == "error")
				logLevel = LogLevel::ERROR;
			else if (levelStr == "fatal")
				logLevel = LogLevel::FATAL;
			else {
				std::cerr << "Unknown log level: " << levelStr << std::endl;
				printUsage(argv[0]);
				return 1;
			}
		} else if (arg.find("--log-file=") == 0) {
			logFilePath = arg.substr(11);
		}
		// PCAP-specific options
		else if (arg.find("--lidar-model=") == 0) {
			config.lidarModel = arg.substr(14);
		} else if (arg.find("--lidar-rpm=") == 0) {
			config.lidarRPM = std::stoi(arg.substr(12));
		} else if (arg.find("--lidar-min-range=") == 0) {
			config.minRange = std::stof(arg.substr(18));
		} else if (arg.find("--lidar-max-range=") == 0) {
			config.maxRange = std::stof(arg.substr(18));
		} else if (arg.find("--oxts-port=") == 0) {
			config.oxtsPort = std::stoi(arg.substr(12));
		} else if (arg == "--read-fast") {
			config.readFast = true;
		} else {
			std::cout << "Unknown option: " << arg << std::endl;
			printUsage(argv[0]);
			return 1;
		}
	}

	Logger::getInstance().init(logFilePath, logLevel);
	LOG_INFO("Starting offline processing");
	LOG_INFO("Input file: " + inputFilePath + " (type: " + fileType + ")");
	LOG_INFO("Output directory: " + outputDir);

	if (config.highQualityMode) {
		config.mappingCornerLeafSize = 0.1;
		config.mappingSurfLeafSize = 0.2;
		std::cout << "High quality mode enabled: Using finer resolution settings" << std::endl;
	}

	try {
		OfflineProcessor processor(config);

		// Load data based on file type
		bool dataLoaded = false;

		if (fileType == "bag") {
			LOG_INFO("Loading bag file...");
			dataLoaded = processor.loadBagFile();
		} else if (fileType == "pcap") {
			LOG_INFO("Loading PCAP file...");
			dataLoaded = processor.loadPcapFile();
		} else {
			LOG_FATAL("Unknown file type: " + fileType);
			return 1;
		}

		if (!dataLoaded) {
			LOG_FATAL("Failed to load input file: " + inputFilePath);
			return 1;
		}

		processor.visualizeOdomTopics(config.outputDirectory);

		LOG_INFO("Processing data...");
		processor.processAllData();

		LOG_INFO("Saving results...");
		if (!processor.saveResults()) {
			LOG_FATAL("Failed to save results");
			return 1;
		}

		LOG_INFO("Processing complete!");
		LOG_INFO("Results saved to: " + outputDir);
	} catch (const std::exception& e) {
		LOG_FATAL(std::string("Error: ") + e.what());
		return 1;
	}

	return 0;
}