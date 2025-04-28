#include "offline_FeatureExtraction.hpp"
#include <algorithm>

OfflineFeatureExtraction::OfflineFeatureExtraction(float edge_threshold, float surf_threshold)
	: edgeThreshold(edge_threshold), surfThreshold(surf_threshold) {
	// Initialize downsampling filter
	downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
}

void OfflineFeatureExtraction::extractFeatures(const pcl::PointCloud<PointType>::Ptr& cloud,
											   const OfflineImageProjection::CloudInfo& cloudInfo,
											   pcl::PointCloud<PointType>::Ptr& cornerCloud,
											   pcl::PointCloud<PointType>::Ptr& surfaceCloud) {
	// TODO: Extract core functionality from featureExtraction.cpp
	// 1. Resize temporary storage
	// 2. Calculate smoothness
	// 3. Mark occluded points
	// 4. Extract corner and surface features

	// Initialize temporary storage
	int cloudSize = cloud->points.size();
	cloudCurvature.resize(cloudSize, 0);
	cloudNeighborPicked.resize(cloudSize, 0);
	cloudLabel.resize(cloudSize, 0);

	// Clear output clouds
	cornerCloud->clear();
	surfaceCloud->clear();

	// Implement the feature extraction stages
	calculateSmoothness();
	markOccludedPoints();
	extractCornerFeatures();
	extractSurfaceFeatures();
}

void OfflineFeatureExtraction::calculateSmoothness() {
	// TODO: Implement smoothness calculation
}

void OfflineFeatureExtraction::markOccludedPoints() {
	// TODO: Implement occluded point marking
}

void OfflineFeatureExtraction::extractCornerFeatures() {
	// TODO: Implement corner feature extraction
}

void OfflineFeatureExtraction::extractSurfaceFeatures() {
	// TODO: Implement surface feature extraction
}