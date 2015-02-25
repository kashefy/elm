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
//    kernel_response.reserve(kernels.size());
//    float norm_factor;  // normalization factor across individual responses

//    for(VecMat1f::const_iterator itr=kernels.begin();
//        itr != kernels.end();
//        itr++) {

//        Mat1f r;
//        cv::filter2D(in, r, -1, *itr, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//        cv::pow(r, 2., r);
//        kernel_response.push_back(r);

//        double min_val, max_val;
//        cv::minMaxIdx(r, &min_val, &max_val);
//        if(norm_factor <= max_val) {

//            norm_factor = max_val;
//        }
//    }
    fan_out_ = in.cols;
    state_.clear();
    for(int r=0; r<in.rows; r++) {
        state_.push_back(in.row(r));
    }

    //State(kernel_response);
}

//void SoftMaxPopulationCode::State(const VecMat1f &in)
//{
//    VecMat1f kernel_response = in;
//    Normalize(kernel_response);

//    state_.clear();
//    fan_out_ = static_cast<int>(kernel_response.size());

//    if(in.size() > 0) {

//        for(size_t i=0; i<in[0].total(); i++) {

//            int r = i / in[0].cols;
//            int c = i % in[0].cols;
//            Mat1f node_state(1, fan_out_);
//            int k=0;
//            for(VecMat1f::const_iterator itr=kernel_response.begin();
//                itr != kernel_response.end();
//                itr++, k++) {

//                node_state(0, k) = (*itr)(r, c);
//            }

//            state_.push_back(node_state);
//        }
//    }
//}

//void SoftMaxPopulationCode::Normalize(VecMat1f &response) const
//{
////    // normalization factor across individual responses
////    float norm_factor;
////    for(VecMat1f::const_iterator itr=response.begin();
////        itr != response.end();
////        itr++) {

////        double min_val, max_val;
////        cv::minMaxIdx(*itr, &min_val, &max_val);
////        if(norm_factor <= max_val) {

////            norm_factor = max_val;
////        }
////    }

////    // normalize individual responses by global factor
////    if(norm_factor != 0) {

////        for(VecMat1f::iterator itr=response.begin();
////            itr != response.end();
////            itr++) {

////            (*itr) /= norm_factor;
////        }
////    }
//}

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
