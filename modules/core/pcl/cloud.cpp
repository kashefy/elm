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
#define ADD_SPECIALIZATION__Mat2PointCloud___(TPoint) template<> boost::shared_ptr<pcl::PointCloud<TPoint > > Mat2PointCloud_(const Mat1f &m) { \
    return Mat2PointCloudTP<TPoint>(m); \
}

ADD_SPECIALIZATION__Mat2PointCloud___(PointXYZ)

// macro for implementing Mat2PointCloud_() specializations
#define ADD_SPECIALIZATION__PointCloud_2Mat__(TPoint) template<> cv::Mat1f PointCloud_2Mat(typename pcl::PointCloud<TPoint >::Ptr &cloud_ptr) {    \
                                                                                                                                        \
    TPoint *points_ptr = cloud_ptr->points.data();                                                                                      \
    return cv::Mat1f(cloud_ptr->height, cloud_ptr->width*static_cast<int>(NbFloats<TPoint>()), reinterpret_cast<float*>(points_ptr));   \
}

ADD_SPECIALIZATION__PointCloud_2Mat__(PointXYZ)

} // namespace sem

CloudXYZPtr sem::Mat2PointCloud(const Mat_<float> &m)
{
    return Mat2PointCloud_<PointXYZ>(m);
}

Mat1f sem::PointCloud2Mat(CloudXYZPtr &cloud_ptr)
{
    return PointCloud_2Mat<PointXYZ>(cloud_ptr);
}

#endif // __WITH_PCL

