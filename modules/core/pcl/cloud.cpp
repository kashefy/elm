#include "core/pcl/cloud.h"

#ifdef __WITH_PCL

#include "core/exception.h"
#include "core/cv/mat_utils.h"
#include "core/pcl/cloud_.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

namespace sem {

// macro for implementing Mat2PointCloud_() specializations
#define ADD_SPECIALIZATION_Mat2PointCloud__(TPoint) template<> boost::shared_ptr<pcl::PointCloud<TPoint > > Mat2PointCloud_(const Mat1f &m) { \
    return Mat2PointCloudTP<TPoint>(m); \
}

ADD_SPECIALIZATION_Mat2PointCloud__(PointXYZ)

} // namespace sem

CloudXYZPtr sem::Mat2PointCloud(const Mat_<float> &m)
{
    return Mat2PointCloud_<PointXYZ>(m);
}

Mat1f sem::PointCloud2Mat(CloudXYZPtr &cloud_ptr)
{
    PointXYZ *points_ptr = cloud_ptr->points.data();
    return Mat1f(cloud_ptr->height, cloud_ptr->width*4, reinterpret_cast<float*>(points_ptr));
}

#endif // __WITH_PCL

