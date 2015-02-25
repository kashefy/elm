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
    :base_PopulationCode()
{
}

void SoftMaxPopulationCode::State(const Mat1f &in)//, const VecMat1f &kernels)
{
    fan_out_ = in.cols;
    state_.clear();
    for(int r=0; r<in.rows; r++) {
        state_.push_back(in.row(r));
    }
}

Mat1f SoftMaxPopulationCode::PopCode()
{
    Mat1f pop_code = Mat1f::zeros(1, fan_out_*static_cast<int>(state_.size()));

    int col = 0;
    for(VecMat1f::const_iterator itr=state_.begin(); itr != state_.end(); itr++, col+=fan_out_) {

        Sampler1D sampler;
        // no sampling for all-zero response
        if(cv::sum(*itr)(0) > 0.f) {

            sampler.pdf(*itr);
            pop_code(col+sampler.Sample()) = 1.f;
        }
    }

    return pop_code;
}
