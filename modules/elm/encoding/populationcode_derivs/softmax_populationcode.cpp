/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/encoding/populationcode_derivs/softmax_populationcode.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/sampler.h"
#include "elm/core/signal.h"
#include "elm/encoding/base_filterbank.h"

using std::string;

using cv::Mat1f;

using namespace elm;

SoftMaxPopulationCode::SoftMaxPopulationCode()
    : base_PopulationCode(),
      min_distr_sum_(0.f)
{
}

void SoftMaxPopulationCode::State(const Mat1f &in)//, const VecMat1f &kernels)
{
    state_ = in;
}

Mat1f SoftMaxPopulationCode::PopCode()
{
    Mat1f pop_code = Mat1f::zeros(state_.size());

    for(int r=0; r<state_.rows; r++) {

        Sampler1D sampler;

        Mat1f pdf = state_.row(r);

        // no sampling for all-zero response
        if(cv::sum(pdf)[0] > min_distr_sum_) {

            sampler.pdf(pdf);
            pop_code(r, sampler.Sample()) = 1.f;
        }
    }

    return pop_code;
}
