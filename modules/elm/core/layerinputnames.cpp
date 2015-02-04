/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/layerinputnames.h"

#include <boost/optional.hpp>

#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/stl/stl.h"

using namespace std;
using namespace elm;

LayerInputNames::LayerInputNames()
{
}

void LayerInputNames::Input(const string &key, const InputName &name)
{
    inputs_[key] = name;
}

InputName LayerInputNames::Input(const string &key) const
{
    string name;
    if(!Find<string>(inputs_, key, name)) {

        ELM_THROW_KEY_ERROR("No name found for input key \'" + key + "\'");
    }
    return name;
}

OptS LayerInputNames::InputOpt(const string &key) const
{
    OptS o;
    string name;
    return Find<string>(inputs_, key, name)? name : o;
}
