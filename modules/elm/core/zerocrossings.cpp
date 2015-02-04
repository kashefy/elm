/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/zerocrossings.h"

#include <opencv2/core.hpp>

using namespace cv;

base_ZeroCrossings::~base_ZeroCrossings()
{
}

base_ZeroCrossings::base_ZeroCrossings()
{
}

ZeroCrossingsDiff::ZeroCrossingsDiff()
    : base_ZeroCrossings()
{
}

void ZeroCrossingsDiff::operator ()(const Mat1f &src, Mat1f &dst) const
{
    if(src.empty()) { return; }

    dst = Mat1f::zeros(src.size());

    Mat1f i0, i1, zc_axis;

    // detect horizontal zero-crossing
    if(src.cols > 1) {
        i0 = src.colRange(0, src.cols-1);
        i1 = src.colRange(1, src.cols);
        multiply(i0, i1, zc_axis);
    }
    else {
        zc_axis = Mat1f::zeros(src.rows, 1);
    }

    dst.colRange(1, src.cols).setTo(1.f, zc_axis < 0);

    if(src.rows > 1) {
        i0 = src.rowRange(0, src.rows-1);
        i1 = src.rowRange(1, src.rows);
        multiply(i0, i1, zc_axis);
    }
    else {
        zc_axis = Mat1f::zeros(src.cols, 1);
    }

    dst.rowRange(1, src.rows).setTo(1.f, zc_axis < 0);
}
