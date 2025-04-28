#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "offline_ImageProjection.hpp"
#include "offline_utility.hpp"

class OfflineFeatureExtraction {
   public:
	OfflineFeatureExtraction(float edge_threshold = 0.1, float surf_threshold = 0.1);

	void extractFeatures(const pcl::PointCloud<PointType>::Ptr& cloud,
						 const OfflineImageProjection::CloudInfo& cloudInfo,
						 pcl::PointCloud<PointType>::Ptr& cornerCloud,
						 pcl::PointCloud<PointType>::Ptr& surfaceCloud);

   private:
	float edgeThreshold;
	float surfThreshold;

	// Downsampling filter
	pcl::VoxelGrid<PointType> downSizeFilter;

	// Temporary storage
	std::vector<float> cloudCurvature;
	std::vector<int> cloudNeighborPicked;
	std::vector<int> cloudLabel;

	// Core algorithm methods
	void calculateSmoothness();
	void markOccludedPoints();
	void extractCornerFeatures();
	void extractSurfaceFeatures();
};