/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/visitors/visitorvecmat1f.h"

#include <opencv2/core/core.hpp>

#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/point_traits.h"
#include "elm/core/pcl/vertices.h"

using namespace cv;
using namespace elm;

extern template class cv::Mat_<float>;
extern template class cv::SparseMat_<float>;

VecMat1f VisitorVecMat1f::operator()(const VecMat1f &v) const
{
    return v;
}

VecMat1f VisitorVecMat1f::operator()(const Mat &m) const
{
    VecMat1f v;
    if((m.type() & CV_MAT_DEPTH_MASK) != (CV_32F & CV_MAT_DEPTH_MASK)) {

        Mat mf;
        m.convertTo(mf, CV_MAKE_TYPE(CV_32F, m.channels()));
        v.push_back(mf);
    }
    else {
        v.push_back(m);
    }
    return v;
}

VecMat1f VisitorVecMat1f::operator()(const Mat1f &m) const
{
    return VecMat1f(1, m);
}

VecMat1f VisitorVecMat1f::operator()(const SparseMat1f &m) const
{
    Mat1f dense;
    if(m.size()!=0) {

        m.convertTo(dense, dense.type());
    }
    return VecMat1f(1, dense);
}

VecMat1f VisitorVecMat1f::operator()(float f) const
{
    return VecMat1f(1, Mat1f(1, 1, f));
}

VecMat1f VisitorVecMat1f::operator()(int n) const
{
    return VecMat1f(1, Mat1f(1, 1, static_cast<float>(n)));
}

VecMat1f VisitorVecMat1f::operator()(uchar c) const
{
    return VecMat1f(1, Mat1f(1, 1, static_cast<float>(c)));
}

#ifdef __WITH_PCL // definitions below require PCL support

VecMat1f VisitorVecMat1f::operator()(const VecVertices &vv) const
{
    VecMat1f v(vv.size());

    for(size_t i=0; i<vv.size(); i++) {

        std::vector<uint32_t> tmp = vv[i].vertices;
        int sz_vertex = static_cast<int>(tmp.size());

        Mat1f m(1, sz_vertex);

        for(int j=0; j<sz_vertex; j++) {

            m(j) = static_cast<float>(tmp[j]);
        }

        v[i] = m;
    }

    return v;
}

template<class TPoint>
void PointCloud2VecMat1f_(boost::shared_ptr<pcl::PointCloud<TPoint > > &cld, VecMat1f &dst)
{
    int cols = PCLPointTraits_<TPoint >::NbFloats();

    int i = 0 ;
    for(typename pcl::PointCloud<TPoint >::iterator itr=cld->begin();
            itr != cld->end(); ++itr, i++) {

        Mat1f m(1, cols, reinterpret_cast<float*>(&(*itr)));
        dst[i] = m;
    }
}

VecMat1f VisitorVecMat1f::operator()(CloudXYZPtr &cld) const
{
    VecMat1f v(cld->size());
    PointCloud2VecMat1f_<pcl::PointXYZ >(cld, v);
    return v;
}

VecMat1f VisitorVecMat1f::operator()(CloudNrmlPtr &cld) const
{
    VecMat1f v(cld->size());
    PointCloud2VecMat1f_<pcl::Normal >(cld, v);
    return v;
}

VecMat1f VisitorVecMat1f::operator()(CloudPtNrmlPtr &cld) const
{
    VecMat1f v(cld->size());
    PointCloud2VecMat1f_<pcl::PointNormal >(cld, v);
    return v;
}

#endif // __WITH_PCL
