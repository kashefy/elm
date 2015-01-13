#include "encoding/populationcode.h"

#include <boost/foreach.hpp> ///< for iterating through param kernels

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "core/sampler.h"
#include "core/signal.h"
#include "encoding/base_filterbank.h"

using cv::Mat1f;

using std::string;

const string base_PopulationCode::KEY_INPUT_STIMULUS   = "in";
const string base_PopulationCode::KEY_OUTPUT_POP_CODE  = "pc";

base_PopulationCode::~base_PopulationCode()
{
}

base_PopulationCode::base_PopulationCode()
    : base_Layer()
{
}

base_PopulationCode::base_PopulationCode(const LayerConfig &config)
    : base_Layer(config)
{
}

void base_PopulationCode::Clear()
{
    pop_code_ = Mat1f();
}

void base_PopulationCode::Reconfigure(const LayerConfig &config)
{
    SEM_THROW_NOT_IMPLEMENTED;
}

void base_PopulationCode::Reset(const LayerConfig &config)
{
    Clear();
}

void base_PopulationCode::IONames(const LayerIONames &config)
{
    name_stimulus_ = config.Input(KEY_INPUT_STIMULUS);
    name_pop_code_ = config.Output(KEY_OUTPUT_POP_CODE);
}

void base_PopulationCode::Activate(const Signal &signal)
{
    State(signal.MostRecentMat(name_stimulus_), VecMat1f());
    pop_code_ = PopCode();
}

void base_PopulationCode::Response(Signal &signal)
{
    signal.Append(name_pop_code_, pop_code_);
}

MutexPopulationCode::MutexPopulationCode()
    : base_PopulationCode()
{
}

MutexPopulationCode::MutexPopulationCode(const LayerConfig &config)
    : base_PopulationCode(config)
{
}

void MutexPopulationCode::State(const Mat1f& in, const VecMat1f& kernels)
{
    pop_code_ = Mat1f::zeros(in.rows, in.cols*2);
    for(int i=0, j=0; i < static_cast<int>(in.total()); i++, j+=2) {

        int k = (in(i) < 0.5f) ? 0 : 1;
        pop_code_(j+k) = 1.f;
    }
}

Mat1f MutexPopulationCode::PopCode()
{
    return pop_code_;
}

const string base_StatefulPopulationCode::KEY_OUTPUT_OPT_STATE = "state";

base_StatefulPopulationCode::base_StatefulPopulationCode()
    : base_PopulationCode()
{
}

base_StatefulPopulationCode::base_StatefulPopulationCode(const LayerConfig &config)
    : base_PopulationCode(config)
{
}

void base_StatefulPopulationCode::Clear()
{
    base_PopulationCode::Clear();
    state_ = Mat1f();
}

void base_StatefulPopulationCode::IONames(const LayerIONames &config)
{
    base_PopulationCode::IONames(config);
    name_state_ = config.OutputOpt(KEY_OUTPUT_OPT_STATE);
}

void base_StatefulPopulationCode::Response(Signal &signal)
{
    base_PopulationCode::Response(signal);
    if(name_state_) {
        signal.Append(name_state_.get(), state_);
    }
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

    for(VecMat1f::const_iterator itr=kernels.begin();
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

        for(VecMat1f::iterator itr=kernel_response.begin();
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
        for(VecMat1f::const_iterator itr=kernel_response.begin();
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
        for(VecMat1f::const_iterator itr=kernel_response.begin();
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
    for(VecMat1f::const_iterator itr=response.begin();
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

        for(VecMat1f::iterator itr=response.begin();
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
