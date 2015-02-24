/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/base_layer_derivations/base_filterbanklayer.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "elm/core/cv/mat_vector_utils.h"
#include "elm/core/layeroutputnames.h"
#include "elm/core/signal.h"

using namespace cv;
using namespace elm;

base_FilterBankLayer::~base_FilterBankLayer()
{
}

base_FilterBankLayer::base_FilterBankLayer()
    : base_SingleInputFeatureLayer()
{
}

base_FilterBankLayer::base_FilterBankLayer(const LayerConfig &cfg)
    : base_SingleInputFeatureLayer(cfg)
{
}

void base_FilterBankLayer::Clear()
{
    kernels_ = VecMat1f();
    response_ = VecMat1f();
}

void base_FilterBankLayer::Reset(const LayerConfig &config)
{
    Clear();
    Reconfigure(config);
    kernels_ = Kernels();
}

void base_FilterBankLayer::OutputNames(const LayerOutputNames &io)
{
    name_out_ = io.Output(KEY_OUTPUT_RESPONSE);
}

void base_FilterBankLayer::Activate(const Signal &signal)
{
    response_.clear();
    response_.reserve(kernels_.size());
    response_ = Convolve(signal.MostRecentMat1f(name_input_));
}

void base_FilterBankLayer::Response(Signal &signal)
{
    signal.Append(name_out_, response_);
}

VecMat1f base_FilterBankLayer::Convolve(const cv::Mat1f &stimulus)
{
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

void base_FilterBankLayer::Rectify(Mat1f &response)
{
// default is to do nothing and leave response as is
}

Mat1f base_FilterBankLayer::ElementResponse(int r, int c) const
{
    return elm::ElementsAt(response_, r, c);
}

size_t base_FilterBankLayer::size() const
{
    return Kernels().size();
}
