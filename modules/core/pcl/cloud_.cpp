#include "core/pcl/cloud_.h"

#ifdef __WITH_PCL

#include "core/exception.h"
#include "core/cv/mat_utils.h"
#include "core/pcl/cloud_impl_.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

namespace sem {

// macro for implementing Mat2PointCloud_() specializations
#define SEM_ADD_SPECIALIZATION__Mat2PointCloud___(TPoint) template<> boost::shared_ptr<pcl::PointCloud<TPoint > > Mat2PointCloud_(const Mat1f &m) { \
    return ConverterCloudMat_<TPoint>::Mat2PointCloud(m); \
}

// macro for implementing Mat2PointCloud_() specializations
#define SEM_ADD_SPECIALIZATION__PointCloud2Mat___(TPoint) template<> cv::Mat1f PointCloud2Mat_(typename pcl::PointCloud<TPoint >::Ptr &cloud_ptr) {    \
                                                                                                                                        \
    TPoint *points_ptr = cloud_ptr->points.data();                                                                                      \
    return cv::Mat1f(cloud_ptr->height, cloud_ptr->width*static_cast<int>(ConverterCloudMat_<TPoint>::NbFloats()), reinterpret_cast<float*>(points_ptr));   \
}

SEM_ADD_SPECIALIZATION__Mat2PointCloud___(PointXYZ)
SEM_ADD_SPECIALIZATION__PointCloud2Mat___(PointXYZ)

SEM_ADD_SPECIALIZATION__Mat2PointCloud___(Normal)
SEM_ADD_SPECIALIZATION__PointCloud2Mat___(Normal)

SEM_ADD_SPECIALIZATION__Mat2PointCloud___(PointNormal)
SEM_ADD_SPECIALIZATION__PointCloud2Mat___(PointNormal)

} // namespace sem

#endif // __WITH_PCL

