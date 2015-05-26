/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layerlist.h"

#include "elm/layers/layergraph.h"

using namespace elm;

LayerGraph::~LayerGraph()
{
    if(impl_ == NULL) {

        delete impl_;
    }
    impl_ = NULL;
}

LayerGraph::LayerGraph()
    : impl_(new LayerGraph_Impl)
{
}
