/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/layeroutputnames.h"

#include <boost/optional.hpp>

#include "elm/core/exception.h"
#include "elm/core/stl/stl_inl.h"

using namespace std;
using namespace elm;

void LayerOutputNames::Output(const string &key, const string &name)
{
    outputs_[key] = name;
}

string LayerOutputNames::Output(const string &key) const
{
    string name;
    if(!Find<string>(outputs_, key, name)) {

        ELM_THROW_KEY_ERROR("No name found for input key \'" + key + "\'");
    }
    return name;
}

OptS LayerOutputNames::OutputOpt(const string &key) const
{
    OptS o;
    string name;
    return Find<string>(outputs_, key, name)? name : o;
}

const MapSS& LayerOutputNames::OutputMap() const {

    return outputs_;
}
