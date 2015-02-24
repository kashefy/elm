/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/visitors/visitorvertices.h"

#ifdef __WITH_PCL // definitions below require PCL support

#include <opencv2/core/core.hpp>

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

VecVertices VisitorVecVertices::operator()(const cv::Mat1f &m)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(m);
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(const SparseMat1f &m)
{
    if(vv_.empty()) {

        cv::Mat1f dense;
        if(m.size()!=0) {

            m.convertTo(dense, dense.type());
        }
        vv_ = Mat2VecVertices(dense);
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(const VecMat1f &v)
{
    if(vv_.empty()) {

        for(size_t i=0; i<v.size(); i++) {

            VecVertices tmp = Mat2VecVertices(v[i].reshape(1, 1));
            vv_.push_back(tmp[0]);
        }
    }
    return vv_;
}

#endif // __WITH_PCL
