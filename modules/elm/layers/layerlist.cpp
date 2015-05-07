/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layerlist.h"

#include "elm/layers/layerlistimpl.h"

using namespace elm;

LayerList::~LayerList()
{
    if(impl_ == NULL) {

        delete impl_;
    }
    impl_ = NULL;
}

LayerList::LayerList()
    : impl_(new LayerListImpl)
{
}
