/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/percentile.h"

#include <opencv2/core/core.hpp>
#include "elm/core/exception.h"

using namespace cv;
using namespace elm;

const float Percentile::MEDIAN = 0.5f;

Percentile::Percentile()
{
}

float Percentile::CalcPercentile(const Mat1f &in, float percentile) const
{
    if(in.empty()) {

        ELM_THROW_BAD_DIMS("Input is empty.");
    }

    if(percentile > 1.f || percentile < 0.f) {

        ELM_THROW_VALUE_ERROR("Requested percentile must be within the [0, 1] range.");
    }

    Mat1f dst;
    // sort and extract
    cv::sort(in, dst, SORT_EVERY_ROW+SORT_ASCENDING);

    int sz_total = static_cast<int>(in.total());

    int pos = static_cast<int>(percentile*sz_total); // equivalent to flooring

    float result;

    if(percentile == MEDIAN && sz_total % 2 == 0) {

        result = (dst(pos-1) + dst(pos))/2.f;
    }
    else {
        result = dst(pos);
    }

    return result;
}
