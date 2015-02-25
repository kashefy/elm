/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/encoding/populationcode_derivs/base_statefulpopulationcode.h"

#include <string>

#include "elm/core/sampler.h"
#include "elm/core/signal.h"

using std::string;

using cv::Mat1f;

using namespace elm;

const string base_StatefulPopulationCode::KEY_OUTPUT_OPT_STATE = "state";

base_StatefulPopulationCode::~base_StatefulPopulationCode()
{
}

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

void base_StatefulPopulationCode::OutputNames(const LayerIONames &config)
{
    base_PopulationCode::OutputNames(config);
    name_state_ = config.OutputOpt(KEY_OUTPUT_OPT_STATE);
}

void base_StatefulPopulationCode::Response(Signal &signal)
{
    base_PopulationCode::Response(signal);
    if(name_state_) {
        signal.Append(name_state_.get(), state_);
    }
}
