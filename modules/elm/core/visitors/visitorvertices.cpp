/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/visitors/visitorvertices.h"

#ifdef __WITH_PCL // definitions below require PCL support

#include <opencv2/core.hpp>

#include "elm/core/pcl/cloud_.h"

using namespace pcl;
using namespace elm;

void VisitorVecVertices::Reset()
{
    vv_.clear();
}

VecVertices VisitorVecVertices::operator()(const VecVertices &vv)
{
    if(vv_.empty()) {

        vv_ = vv;
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(CloudXYZPtr &c)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(PointCloud2Mat_<PointXYZ>(c)); // or keep cache?
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(CloudNrmlPtr &c)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(PointCloud2Mat_<Normal>(c)); // or keep cache?
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(CloudPtNrmlPtr &c)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(PointCloud2Mat_<PointNormal>(c)); // or keep cache?
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(float f)
{
    FromScalar(f);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(int n)
{
    FromScalar(n);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(uchar c)
{
    FromScalar(c);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(const Mat_f &m)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(m);
    }
    return vv_;
}

#endif // __WITH_PCL
