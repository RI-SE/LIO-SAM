#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <vector>
#include "offline_utility.hpp"

class OfflineMapOptimization {
   public:
	// Define pose type for pose graph
	struct PointTypePose {
		double x, y, z;
		double roll, pitch, yaw;
		double time;
	};

	OfflineMapOptimization(float mapping_corner_leaf_size = 0.2,
						   float mapping_surf_leaf_size = 0.4,
						   float surroundingKeyframeSearchRadius = 50.0);

	void addKeyFrame(const pcl::PointCloud<PointType>::Ptr& cornerCloud,
					 const pcl::PointCloud<PointType>::Ptr& surfCloud,
					 double timestamp,
					 const Eigen::Matrix4f& initialGuess);

	void optimizeMap();

	bool saveMap(const std::string& filePath);

	std::vector<PointTypePose> getTrajectory() const;

	// Get the current global map
	pcl::PointCloud<PointType>::Ptr getGlobalMap();

	pcl::PointCloud<PointType>::Ptr transformPointCloud(const pcl::PointCloud<PointType>::Ptr& cloudIn,
														const PointTypePose& pose);

   private:
	// GTSAM objects
	gtsam::NonlinearFactorGraph gtSAMgraph;
	gtsam::Values initialEstimate;
	gtsam::ISAM2* isam;
	gtsam::Values isamCurrentEstimate;

	// Point cloud storage
	std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
	std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
	pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
	std::vector<PointTypePose> cloudKeyPoses6D;

	// For map management
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;	 // Make this a pointer
	pcl::VoxelGrid<PointType> downSizeFilterCorner;				 // Rename to match cpp
	pcl::VoxelGrid<PointType> downSizeFilterSurf;				 // Rename to match cpp

	// Parameters
	float mappingCornerLeafSize;
	float mappingSurfLeafSize;
	float surroundingKeyframeSearchRadius;

	// Core algorithm methods
	void addOdomFactor();
	void addGPSFactor();
	void addLoopFactor();
	void correctPoses();
	bool saveKeyFramesAndFactor();
	void updatePath();
	void extractSurroundingKeyFrames();
	void downsampleCurrentScan();
	void scan2MapOptimization();
};