#include "eval/reversecorr.h"

#include <opencv2/imgproc.hpp>

#include "core/exception.h"

using namespace cv;

base_ReverseCorr::~base_ReverseCorr()
{

}

base_ReverseCorr::base_ReverseCorr()
{
}

STA::STA()
    : base_ReverseCorr(),
      nb_samples_(0)
{
}

void STA::Add(InputArray in)
{
    if(sta_.empty()) {

        if(in.empty()) {
            SEM_THROW_BAD_DIMS("Initial input is empty.");
        }
        else {
            sta_ = Mat1f::zeros(in.rows(), in.cols());
        }
    }
    else if(sta_.size() != in.size())
    {
        SEM_THROW_BAD_DIMS("STA input dims have changed from initial input.");
    }

    Mat non_zero;
    compare(in, 0.5f, non_zero, CMP_GE);

    add(sta_, 1, sta_, non_zero);
    nb_samples_++;
}

Mat1f STA::Compute() const
{
    if(nb_samples_ > 0) {
        return sta_ / static_cast<float>(nb_samples_);
    }
    else return sta_;
}
