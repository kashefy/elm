#include "encoding/base_filterbank.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "core/mat_utils.h"

using namespace cv;

base_FilterBank::~base_FilterBank()
{
}

base_FilterBank::base_FilterBank()
{
}

VecMat1f base_FilterBank::Compute(Mat1f stimulus)
{
    response_.clear();
    response_.reserve(kernels_.size());

    for(VecMat1f::const_iterator itr=kernels_.begin();
     itr != kernels_.end();
     itr++) {

         Mat1f r;
         filter2D(stimulus, r, -1, *itr, Point(-1, -1), 0, BORDER_REPLICATE);
         Rectify(r);
         response_.push_back(r);
     }
    return response_;
}

VecMat1f base_FilterBank::Response() const
{
    return response_;
}

void base_FilterBank::Rectify(Mat1f &response)
{
// default is to do nothing and leave response as is
}

Mat1f base_FilterBank::ElementResponse(int r, int c) const
{
    return sem::ElementsAt(response_, r, c);
}

size_t base_FilterBank::size() const
{
    return Kernels().size();
}
