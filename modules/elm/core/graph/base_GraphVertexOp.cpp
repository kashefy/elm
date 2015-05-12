/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/base_GraphVertexOp.h"
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace elm;

extern template class cv::Mat_<float>;
extern template class cv::Mat_<int>;

boost::mutex base_GraphVertexOp::mtx_;

base_GraphVertexOp::~base_GraphVertexOp()
{
}

base_GraphVertexOp::base_GraphVertexOp()
{
}

void base_GraphVertexOp::mutableOpCaller(const Mat1i &img, const Mat &mask, Mat1f &dst)
{
//    ELM_COUT_VAR(""<<img);
//    ELM_COUT_VAR(""<<mask);
    dst = mutableOp(img, mask);
}
