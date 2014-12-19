#include "encoding/populationcode.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "core/sampler.h"
#include "encoding/base_filterbank.h"

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
    state_ = Mat1f::zeros(in.rows, in.cols*2);
    for(int i=0, j=0; i < static_cast<int>(in.total()); i++, j+=2) {

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
    VecMat1f kernel_response;
    kernel_response.reserve(kernels.size());
    float norm_factor;  // normalization factor across individual responses

    for(VecMat1fCIter itr=kernels.begin();
        itr != kernels.end();
        itr++) {

        Mat1f r;
        cv::filter2D(in, r, -1, *itr, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
        cv::pow(r, 2., r);
        kernel_response.push_back(r);

        double min_val, max_val;
        cv::minMaxIdx(r, &min_val, &max_val);
        if(norm_factor <= max_val) {

            norm_factor = max_val;
        }
    }

    // normalize individual responses by global factor
    if(norm_factor != 0) {

        for(VecMat1fIter itr=kernel_response.begin();
            itr != kernel_response.end();
            itr++) {

            (*itr) /= norm_factor;
        }
    }

    state_.clear();
    state_.reserve(in.total());
    fan_out_ = static_cast<int>(kernels.size());
    for(size_t i=0; i<in.total(); i++) {

        int r = i / in.cols;
        int c = i % in.cols;
        Mat1f node_state(1, fan_out_);
        int k=0;
        for(VecMat1fCIter itr=kernel_response.begin();
            itr != kernel_response.end();
            itr++, k++) {

            node_state(0, k) = (*itr)(r, c);
        }
        state_.push_back(node_state);
    }
}

void SoftMaxPopulationCode::State(const Mat1f &in, const std::unique_ptr<base_FilterBank> &filter_bank)
{
    VecMat1f kernel_response = filter_bank->Compute(in);

    Normalize(kernel_response);

    state_.clear();
    state_.reserve(in.total());
    fan_out_ = static_cast<int>(kernel_response.size());

    for(size_t i=0; i<in.total(); i++) {

        int r = i / in.cols;
        int c = i % in.cols;
        Mat1f node_state(1, fan_out_);
        int k=0;
        for(VecMat1fCIter itr=kernel_response.begin();
            itr != kernel_response.end();
            itr++, k++) {

            node_state(0, k) = (*itr)(r, c);
        }

        state_.push_back(node_state);
    }
}

void SoftMaxPopulationCode::Normalize(VecMat1f &response) const
{
    // normalization factor across individual responses
    float norm_factor;
    for(VecMat1fCIter itr=response.begin();
        itr != response.end();
        itr++) {

        double min_val, max_val;
        cv::minMaxIdx(*itr, &min_val, &max_val);
        if(norm_factor <= max_val) {

            norm_factor = max_val;
        }
    }

    // normalize individual responses by global factor
    if(norm_factor != 0) {

        for(VecMat1fIter itr=response.begin();
            itr != response.end();
            itr++) {

            (*itr) /= norm_factor;
        }
    }
}

Mat1f SoftMaxPopulationCode::PopCode()
{
    Mat1f pop_code = Mat1f::zeros(1, fan_out_*static_cast<int>(state_.size()));

    int col = 0;
    for(VecMat1fCIter itr=state_.begin(); itr != state_.end(); itr++, col+=fan_out_) {

        Sampler1D sampler;
        // no sampling for all-zero response
        if(cv::sum(*itr)(0) > 0.f) {

            sampler.pdf(*itr);
            pop_code(col+sampler.Sample()) = 1.f;
        }
    }

    return pop_code;
}
