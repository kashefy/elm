/** A collection of routines to facilitate working with PCL data types.
 * Only defined if PCL support exists.
 */
#ifndef SEM_CORE_PCL_UTILS_H_
#define SEM_CORE_PCL_UTILS_H_

#ifndef __WITH_PCL
    #warning "Skipping pcl utilities since PCL support is disabled."
#else // __WITH_PCL

#include <opencv2/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace sem {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

/**
 * @brief Convert Mat of floats to XYZ point cloud.
 *
 * Involves a deep copy of matrix elements
 *
 * @param source matrix
 * @return pointer to point cloud instance
 * @todo Avoid deep copy
 */
PointCloudXYZ::Ptr Mat2PointCloud(const cv::Mat_<float> &m);

/**
 * @brief Convert XYZ point cloud to OpenCv Mat of floats.
 *
 * @param source point cloud
 * @return resulting Mat
 */
cv::Mat1f PointCloud2Mat(PointCloudXYZ::Ptr &cloud_ptr);

}

#endif // __WITH_PCL

#endif // SEM_CORE_PCL_UTILS_H_
