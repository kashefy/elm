/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/layerionames.h"

#include <boost/optional.hpp>

#include "elm/core/stl/stl.h"
#include "elm/core/exception.h"

using namespace std;
using namespace elm;

void LayerIONames::Input(const string &key, const InputName &name)
{
    inputs_[key] = name;
}

void LayerIONames::Output(const string &key, const string &name)
{
    outputs_[key] = name;
}

InputName LayerIONames::Input(const string &key) const
{
    string name;
    if(!Find<string>(inputs_, key, name)) {

        ELM_THROW_KEY_ERROR("No name found for input key \'" + key + "\'");
    }
    return name;
}

OptS LayerIONames::InputOpt(const string &key) const
{
    OptS o;
    string name;
    return Find<string>(outputs_, key, name)? name : o;
}

string LayerIONames::Output(const string &key) const
{
    string name;
    if(!Find<string>(outputs_, key, name)) {

        ELM_THROW_KEY_ERROR("No name found for input key \'" + key + "\'");
    }
    return name;
}

OptS LayerIONames::OutputOpt(const string &key) const
{
    OptS o;
    string name;
    return Find<string>(outputs_, key, name)? name : o;
}
