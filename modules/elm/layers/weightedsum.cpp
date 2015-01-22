/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/weightedsum.h"

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"

using namespace std;
using namespace cv;

/** Define parameters and I/O keys
  */
const std::string WeightedSum::PARAM_A              = "a";
const std::string WeightedSum::PARAM_B              = "b";
const std::string WeightedSum::KEY_INPUT_STIMULUS   = "in";
const std::string WeightedSum::KEY_OUTPUT_RESPONSE  =  "out";

void WeightedSum::Clear()
{
    response_ = Mat1f();
}

void WeightedSum::Reset(const LayerConfig &config)
{
    Reconfigure(config);
}

void WeightedSum::Reconfigure(const LayerConfig &config)
{
    // params
    PTree params = config.Params();
    a_ = params.get<float>(PARAM_A);
    b_ = params.get<float>(PARAM_B);
}

void WeightedSum::IONames(const LayerIONames &config)
{
    name_stimulus_ = config.Input(KEY_INPUT_STIMULUS);
    name_response_ = config.Output(KEY_OUTPUT_RESPONSE);
}

void WeightedSum::Activate(const Signal &signal)
{
    Mat1f stimulus = signal[name_stimulus_][0];
    if(stimulus.cols > 2) {

        ELM_THROW_BAD_DIMS("Cannot handle stimulus with > 2 columns.");
    }

    response_ = Mat1f(stimulus.rows, 1);
    for(int r=0; r<stimulus.rows; r++) {

        float tmp = a_ * stimulus(r, 0);
        if(stimulus.cols > 1) {

            tmp += b_ * stimulus(r, 1);
        }
        response_(r) = tmp;
    }
}

void WeightedSum::Response(Signal &signal)
{
    signal.Append(name_response_, response_);
}

WeightedSum::WeightedSum()
    : base_Layer()
{
    Clear();
}

WeightedSum::WeightedSum(const LayerConfig& config)
    : base_Layer(config)
{
    Clear();
    Reconfigure(config);
    IONames(config);
}
