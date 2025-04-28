#include "offline_utility.hpp"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

template class pcl::VoxelGrid<PointXYZIRTRGB>;
template class pcl::Filter<PointXYZIRTRGB>;
template class pcl::PCLBase<PointXYZIRTRGB>;
template class pcl::KdTreeFLANN<PointXYZIRTRGB>;
