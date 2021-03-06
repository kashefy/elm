/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/visitors/visitorsparsemat1f.h"

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/vertices.h"

using namespace cv;
using namespace elm;

extern template class cv::Mat_<float>;
extern template class cv::SparseMat_<float>;
extern template class std::vector<cv::Mat_<float> >;

SparseMat1f VisitorSparseMat1f::operator()(const SparseMat1f &m) const
{
    return m;
}

SparseMat1f VisitorSparseMat1f::operator()(const Mat &m) const
{
    ELM_THROW_BAD_DIMS_IF(m.empty(), "Cannot create a sparse matrix from empty Mat");
    if((m.type() & CV_MAT_DEPTH_MASK) != (CV_32F & CV_MAT_DEPTH_MASK)) {

        Mat mf;
        m.convertTo(mf, CV_MAKE_TYPE(CV_32F, m.channels()));
        return mf;
    }
    else {
        return m;
    }
}

SparseMat1f VisitorSparseMat1f::operator()(const Mat1f &m) const
{
    return (m.empty())? SparseMat1f() : SparseMat1f(m);
}

SparseMat1f VisitorSparseMat1f::operator()(const VecMat1f &v) const
{
    SparseMat sparse;

    size_t n = v.size();

    if(n > 0) {

        Mat1f dense;
        for(size_t i=0; i<n; i++) {

            ELM_THROW_BAD_DIMS_IF(!dense.empty() && v[i].cols != dense.cols,
                                  "vector element dims must stay consistent.");
            dense.push_back(v[i]);
        }
        sparse = operator ()(dense);
    }
    else {

        sparse = SparseMat1f();
    }

    return sparse;
}

SparseMat1f VisitorSparseMat1f::operator()(float f) const
{
    return SparseMat1f(Mat1f(1, 1, f));
}

SparseMat1f VisitorSparseMat1f::operator()(int n) const
{
    return SparseMat1f(Mat1f(1, 1, static_cast<float>(n)));
}

SparseMat1f VisitorSparseMat1f::operator()(uchar c) const
{
    return SparseMat1f(Mat1f(1, 1, static_cast<float>(c)));
}

#ifdef __WITH_PCL // definitions below require PCL support

SparseMat1f VisitorSparseMat1f::operator()(const VecVertices &vv) const
{
    return this->operator ()(VecVertices2Mat(vv, false));
}

SparseMat1f VisitorSparseMat1f::operator()(CloudXYZPtr &c) const
{
    return this->operator ()(PointCloud2Mat_<pcl::PointXYZ>(c));
}

SparseMat1f VisitorSparseMat1f::operator()(CloudNrmlPtr &c) const
{
    return this->operator ()(PointCloud2Mat_<pcl::Normal>(c));
}

SparseMat1f VisitorSparseMat1f::operator()(CloudPtNrmlPtr &c) const
{
    return this->operator ()(PointCloud2Mat_<pcl::PointNormal>(c));
}

#endif // __WITH_PCL
