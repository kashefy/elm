/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/encoding/base_populationcode.h"

#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerinputnames.h"
#include "elm/core/layeroutputnames.h"
#include "elm/core/sampler.h"
#include "elm/core/signal.h"

using std::string;

using cv::Mat1f;

using namespace elm;

const string base_PopulationCode::KEY_INPUT_STIMULUS   = detail::BASE_POPULATIONCODE__KEY_INPUT_STIMULUS;
const string base_PopulationCode::KEY_OUTPUT_POP_CODE  = detail::BASE_POPULATIONCODE__KEY_OUTPUT_POP_CODE;

base_PopulationCode::~base_PopulationCode()
{
}

base_PopulationCode::base_PopulationCode()
    : base_Layer()
{
}

void base_PopulationCode::Clear()
{
    pop_code_ = Mat1f();
}

void base_PopulationCode::Reconfigure(const LayerConfig &config)
{
    ELM_THROW_NOT_IMPLEMENTED;
}

void base_PopulationCode::Reset(const LayerConfig &config)
{
    Clear();
}

void base_PopulationCode::InputNames(const LayerInputNames &config)
{
    name_stimulus_ = config.Input(KEY_INPUT_STIMULUS);
}

void base_PopulationCode::OutputNames(const LayerOutputNames &config)
{
    name_pop_code_ = config.Output(KEY_OUTPUT_POP_CODE);
}

void base_PopulationCode::Activate(const Signal &signal)
{
    State(signal.MostRecentMat1f(name_stimulus_));
    pop_code_ = PopCode();
}

void base_PopulationCode::Response(Signal &signal)
{
    signal.Append(name_pop_code_, pop_code_);
}

