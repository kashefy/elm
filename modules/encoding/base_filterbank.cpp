#include "encoding/base_filterbank.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

base_FilterBank::~base_FilterBank()
{
}

base_FilterBank::base_FilterBank()
{
}

VecMat1f base_FilterBank::Compute(cv::Mat1f stimulus)
{
    response_.clear();
    response_.reserve(kernels_.size());

    for(VecMat1fCIter itr=kernels_.begin();
     itr != kernels_.end();
     itr++) {

         Mat1f r;
         filter2D(stimulus, r, -1, *itr, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
         pow(r, 2., r);
         response_.push_back(r);
     }
    return response_;
}

Mat1f base_FilterBank::ElementResponse(int row, int col)
{
    int fan_out = static_cast<int>(kernels_.size());
    Mat1f el_response(1, fan_out);
    int k=0;
    for(VecMat1fCIter itr=response_.begin();
        itr != response_.end();
        itr++, k++) {

        el_response(k) = (*itr)(row, col);
    }

    return el_response;
}
