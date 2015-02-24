/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/visitors/visitormat1f.h"

#include <opencv2/core/core.hpp>

#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/vertices.h"

using namespace cv;
using namespace elm;

Mat1f VisitorMat1f::operator()(const Mat &m) const
{
    if((m.type() & CV_MAT_DEPTH_MASK) != (CV_32F & CV_MAT_DEPTH_MASK)) {

        Mat mf;
        m.convertTo(mf, CV_MAKE_TYPE(CV_32F, m.channels()));
        return mf;
    }
    else {
        return m;
    }
}

Mat1f VisitorMat1f::operator()(const Mat1f &m) const
{
    return m;
}

Mat1f VisitorMat1f::operator()(const SparseMat1f &m) const
{
    Mat1f dense;
    if(m.size()!=0) {

        m.convertTo(dense, dense.type());
    }
    return dense;
}

Mat1f VisitorMat1f::operator()(const elm::VecMat1f &v) const
{
    Mat1f m;
    for(size_t i=0; i<v.size(); i++) {

        m.push_back(v[i]);
    }
    return m;
}

Mat1f VisitorMat1f::operator()(float f) const
{
    return Mat1f(1, 1, f);
}

Mat1f VisitorMat1f::operator()(int n) const
{
    return Mat1f(1, 1, static_cast<float>(n));
}

Mat1f VisitorMat1f::operator()(uchar c) const
{
    return Mat1f(1, 1, static_cast<float>(c));
}

#ifdef __WITH_PCL // definitions below require PCL support

Mat1f VisitorMat1f::operator()(const VecVertices &vv) const
{
    return VecVertices2Mat(vv, false);
}

Mat1f VisitorMat1f::operator()(CloudXYZPtr &c) const
{
    return PointCloud2Mat_<pcl::PointXYZ>(c);
}

Mat1f VisitorMat1f::operator()(CloudNrmlPtr &c) const
{
    return PointCloud2Mat_<pcl::Normal>(c);
}

Mat1f VisitorMat1f::operator()(CloudPtNrmlPtr &c) const
{
    return PointCloud2Mat_<pcl::PointNormal>(c);
}

#endif // __WITH_PCL
