/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layers_interim/base_filterbanklayer.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "elm/core/cv/mat_vector_utils.h"
#include "elm/core/layeroutputnames.h"
#include "elm/core/signal.h"

using std::string;
using namespace cv;
using namespace elm;

const string base_FilterBankLayer::KEY_OUTPUT_RESPONSE = detail::BASE_FILTERBANK_LAYER__KEY_OUTPUT_RESPONSE;

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
    response_ = Convolve(signal.MostRecentMat1f(name_input_));
}

void base_FilterBankLayer::Response(Signal &signal)
{
    signal.Append(name_out_, response_);

}

