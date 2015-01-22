#include "elm/core/percentile.h"

#include "elm/core/exception.h"

using namespace cv;

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
    int pos = static_cast<int>(percentile*in.total()); // equivalent to flooring, should we round?

    return dst(pos);
}
