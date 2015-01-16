/** A collection of routines to facilitate working with PCL data types.
 * Only defined if PCL support exists.
 */
#ifndef SEM_CORE_PCL_CLOUD_H_
#define SEM_CORE_PCL_CLOUD_H_

#ifndef __WITH_PCL
    #warning "Skipping pcl utilities since PCL support is disabled."
#else // __WITH_PCL

// any user will need complete type definitions, so let's provide them
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "core/cv/typedefs_fwd.h"
#include "core/pcl/typedefs_fwd.h"

namespace sem {

/**
 * @brief Convert Mat of floats to XYZ point cloud.
 *
 * Involves a deep copy of matrix elements
 *
 * @param source matrix
 * @return pointer to point cloud instance
 * @todo Avoid deep copy, add test coverage for 4-channel Mat
 */
CloudXYZPtr Mat2PointCloud(const cv::Mat1f &m);

template <class TPoint>
boost::shared_ptr<pcl::PointCloud<TPoint > > Mat2PointCloud_(const cv::Mat1f &m);

/**
 * @brief Convert XYZ point cloud to OpenCV's Mat of floats.
 *
 * @param source point cloud
 * @return resulting Mat
 */
cv::Mat1f PointCloud2Mat(CloudXYZPtr &cloud_ptr);

template <class TPoint>
cv::Mat1f PointCloud_2Mat(boost::shared_ptr<pcl::PointCloud<TPoint > > &cloud_ptr);

} // namespace sem

#endif // __WITH_PCL

#endif // SEM_CORE_PCL_CLOUD_H_
