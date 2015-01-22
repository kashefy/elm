/** A collection of routines to facilitate working with PCL data types.
 * Only defined if PCL support exists.
 */
#ifndef ELM_CORE_PCL_CLOUD__H_
#define ELM_CORE_PCL_CLOUD__H_

#ifndef __WITH_PCL
    #warning "Skipping pcl utilities since PCL support is disabled."
#else // __WITH_PCL

// any user will need complete type definitions, so let's provide them
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "sem/core/cv/typedefs_fwd.h"
#include "sem/core/pcl/typedefs_fwd.h"

namespace elm {

/**
 * @brief Convert a single-channel Mat of floats to point cloud.
 *
 * Involves a deep copy of matrix elements
 *
 * Types for which specializations exist:
 * pcl::PointXYZ, pcl::Normal, pcl::PointNormal
 *
 * @param source matrix
 * @return pointer to point cloud instance
 *
 * @todo Avoid deep copy
 * @todo produce clearer error messages for types without specializations
 */
template <class TPoint>
boost::shared_ptr<pcl::PointCloud<TPoint > > Mat2PointCloud_(const cv::Mat1f &m);

/**
 * @brief Convert point cloud to OpenCV's single-channel Mat of floats.
 *
 * No deep copy involved, ownership of underlying data remains with point cloud.
 *
 * Types for which specializations exist:
 * pcl::PointXYZ, pcl::Normal, pcl::PointNormal
 *
 * @param source point cloud
 * @return resulting Mat
 * @todo produce clearer error messages for types without specializations
 */
template <class TPoint>
cv::Mat1f PointCloud2Mat_(boost::shared_ptr<pcl::PointCloud<TPoint > > &cloud_ptr);

} // namespace elm

#endif // __WITH_PCL

#endif // ELM_CORE_PCL_CLOUD__H_
