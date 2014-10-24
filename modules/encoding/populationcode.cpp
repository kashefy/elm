#include "encoding/populationcode.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

using cv::Mat1f;

base_PopulationCode::base_PopulationCode()
{
}

MutexPopulationCode::MutexPopulationCode()
    : base_PopulationCode()
{
}

void MutexPopulationCode::State(const Mat1f& in, const VecMat1f& kernels)
{
    state_ = Mat1f::zeros(in.rows*2, 1);
    for(int i=0, j=0; i < in.rows; i++, j+=2) {

        int k = (in(i) < 0.5f) ? 0 : 1;
        state_(j+k) = 1.f;
    }
}

Mat1f MutexPopulationCode::PopCode()
{
    return state_;
}

SoftMaxPopulationCode::SoftMaxPopulationCode()
    :base_PopulationCode()
{
}

void SoftMaxPopulationCode::State(const cv::Mat1f &in, const VecMat1f &kernels)
{
    const int NB_KERNELS = static_cast<int>(kernels.size());
    VecMat1f r;
    r.reserve(NB_KERNELS);
    for(int k=0; k<NB_KERNELS; k++) {

        cv::filter2D(in, r[k], 0, kernels[k]);
    }
}

cv::Mat1f SoftMaxPopulationCode::PopCode()
{
    return Mat1f();
}
