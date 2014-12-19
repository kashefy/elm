#include "core/hist.h"

#include <opencv2/imgproc.hpp>

#include "core/exception.h"

using namespace cv;

Hist1Ch::~Hist1Ch()
{
}

Hist1Ch::Hist1Ch()
{
    Reconfigure(10, std::make_pair<float, float>(0., 255.), true, false);
}

void Hist1Ch::Reconfigure(int size, const std::pair<float, float> &range, bool do_uniform, bool do_accumulate)
{
    if(size < 0) { SEM_THROW_VALUE_ERROR("size must be >= 0"); }
    size_ = size;

    range_ = range;

    do_uniform_ = do_uniform;
    do_accumulate_ = do_accumulate;
}

Mat Hist1Ch::Compute(const cv::Mat &in, InputArray mask)
{
    /// Set the ranges
    float range[2] = { range_.first, range_.second } ;
    const float* hist_range = { range };

    calcHist(&in, 1, 0, mask, hist_, 1, &size_, &hist_range,
             do_uniform_, do_accumulate_);

    return hist_.t();
}
