/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/pcl/cloud_.h"

#ifdef __WITH_PCL

#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils.h"
#include "elm/core/pcl/cloud_impl_.h"
#include "elm/core/pcl/point_traits.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

template class pcl::PointCloud<pcl::PointXYZ >;
template class pcl::PointCloud<pcl::Normal >;
template class pcl::PointCloud<pcl::PointNormal >;

namespace elm {

// macro for implementing Mat2PointCloud_() specializations
#define ELM_ADD_SPECIALIZATION__Mat2PointCloud___(TPoint) template<> boost::shared_ptr<pcl::PointCloud<TPoint > > Mat2PointCloud_(const Mat1f &m) { \
    return ConverterCloudMat_<TPoint>::Mat2PointCloud(m); \
}

// macro for implementing Mat2PointCloud_() specializations
#define ELM_ADD_SPECIALIZATION__PointCloud2Mat___(TPoint) template<> cv::Mat1f PointCloud2Mat_(typename pcl::PointCloud<TPoint >::Ptr &cloud_ptr) {    \
                                                                                                                                        \
    TPoint *points_ptr = cloud_ptr->points.data();                                                                                      \
    return cv::Mat1f(cloud_ptr->height, cloud_ptr->width*static_cast<int>(PCLPointTraits_<TPoint>::NbFloats()), reinterpret_cast<float*>(points_ptr));   \
}

ELM_ADD_SPECIALIZATION__Mat2PointCloud___(PointXYZ)
ELM_ADD_SPECIALIZATION__PointCloud2Mat___(PointXYZ)

ELM_ADD_SPECIALIZATION__Mat2PointCloud___(Normal)
ELM_ADD_SPECIALIZATION__PointCloud2Mat___(Normal)

ELM_ADD_SPECIALIZATION__Mat2PointCloud___(PointNormal)
ELM_ADD_SPECIALIZATION__PointCloud2Mat___(PointNormal)
// no multi-channel specialization for PointNormal, channels would take up > 4.

} // namespace elm

#endif // __WITH_PCL

