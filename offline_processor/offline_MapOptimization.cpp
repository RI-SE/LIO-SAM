#include "offline_MapOptimization.hpp"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace gtsam;

OfflineMapOptimization::OfflineMapOptimization(float mapping_corner_leaf_size,
											   float mapping_surf_leaf_size,
											   float surroundingKeyframeSearchRadius)
	: mappingCornerLeafSize(mapping_corner_leaf_size),
	  mappingSurfLeafSize(mapping_surf_leaf_size),
	  surroundingKeyframeSearchRadius(surroundingKeyframeSearchRadius) {
	// Initialize ISAM2
	ISAM2Params parameters;
	parameters.relinearizeThreshold = 0.1;
	parameters.relinearizeSkip = 1;
	isam = new ISAM2(parameters);

	// Initialize the member variables
	cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
	kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

	// Define VoxelGrid filters for downsampling
	downSizeFilterCorner.setLeafSize(mapping_corner_leaf_size, mapping_corner_leaf_size,
									 mapping_corner_leaf_size);
	downSizeFilterSurf.setLeafSize(mapping_surf_leaf_size, mapping_surf_leaf_size, mapping_surf_leaf_size);
}

void OfflineMapOptimization::addKeyFrame(const pcl::PointCloud<PointType>::Ptr& cornerCloud,
										 const pcl::PointCloud<PointType>::Ptr& surfCloud,
										 double timestamp,
										 const Eigen::Matrix4f& initialGuess) {
	// TODO: Implement key frame addition
	// 1. Extract pose from initial guess
	// 2. Add odometry factor
	// 3. Add to the factor graph
	// 4. Optimize the pose graph
	// 5. Save key frames and update trajectory
}

void OfflineMapOptimization::optimizeMap() {
	// TODO: Implement final map optimization
	// 1. Perform loop closure if needed
	// 2. Run final optimization
	// 3. Correct poses
	// 4. Update global map
}

bool OfflineMapOptimization::saveMap(const std::string& filePath) {
	// TODO: Implement map saving
	// 1. Extract global point cloud
	// 2. Downsample if needed
	// 3. Save to PCD file

	std::cout << "Saving map to: " << filePath << std::endl;

	pcl::PointCloud<PointType>::Ptr globalMap(new pcl::PointCloud<PointType>());

	// Combine all keyframes to create the global map
	for (size_t i = 0; i < cloudKeyPoses6D.size(); i++) {
		// Transform point clouds to global frame
		*globalMap += *transformPointCloud(cornerCloudKeyFrames[i], cloudKeyPoses6D[i]);
		*globalMap += *transformPointCloud(surfCloudKeyFrames[i], cloudKeyPoses6D[i]);
	}

	// Optionally downsample the map
	pcl::PointCloud<PointType>::Ptr globalMapDS(new pcl::PointCloud<PointType>());
	pcl::VoxelGrid<PointType> downSizeFilterGlobalMap;
	downSizeFilterGlobalMap.setLeafSize(0.2, 0.2, 0.2);
	downSizeFilterGlobalMap.setInputCloud(globalMap);
	downSizeFilterGlobalMap.filter(*globalMapDS);

	// Save the point cloud to file
	return pcl::io::savePCDFileBinary(filePath, *globalMapDS) == 0;
}

std::vector<OfflineMapOptimization::PointTypePose> OfflineMapOptimization::getTrajectory() const {
	return cloudKeyPoses6D;
}

pcl::PointCloud<PointType>::Ptr OfflineMapOptimization::getGlobalMap() {
	pcl::PointCloud<PointType>::Ptr globalMap(new pcl::PointCloud<PointType>());

	// TODO: Implement global map extraction
	// ...

	return globalMap;
}

// Helper function to transform point cloud using pose
pcl::PointCloud<PointType>::Ptr OfflineMapOptimization::transformPointCloud(
	const pcl::PointCloud<PointType>::Ptr& cloudIn,
	const PointTypePose& pose) {
	pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

	// TODO: Implement point cloud transformation
	// ...

	return cloudOut;
}

// Other private methods to be implemented...