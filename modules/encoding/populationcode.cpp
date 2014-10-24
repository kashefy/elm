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

void SoftMaxPopulationCode::State(const Mat1f &in, const VecMat1f &kernels)
{
    VecMat1f fb_response;
    fb_response.reserve(kernels.size());
    float norm_factor;  // normalization factor across individual responses

    for(VecMat1f::const_iterator itr=kernels.begin();
        itr != kernels.end();
        itr++) {

        Mat1f r;
        cv::filter2D(in, r, 0, *itr);
        fb_response.push_back(r);

        double min_val, max_val;
        cv::minMaxIdx(r, &min_val, &max_val);
        if(norm_factor <= max_val) {

            norm_factor = max_val;
        }
    }

    // normalize individual responses by global factor
    if(norm_factor != 0) {

        for(VecMat1f::iterator itr=fb_response.begin();
            itr != fb_response.end();
            itr++) {

            (*itr) /= norm_factor;
        }
    }

    state_.clear();
    state_.reserve(in.total());
    const int NB_KERNELS=static_cast<int>(kernels.size());
    for(size_t i=0; i<in.total(); i++) {

        Mat1f node_state(1, NB_KERNELS);
        int k=0;
        for(VecMat1f::const_iterator itr=fb_response.begin();
            itr != fb_response.end();
            itr++, k++) {

            node_state(k) = (*itr)(k);
        }
        state_.push_back(node_state);
    }
}

Mat1f SoftMaxPopulationCode::PopCode()
{
    return Mat1f();
}
