/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layergraph.h"

#include "elm/layers/layergraph_impl.h"

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

void LayerGraph::Add(const std::string &name,
                     LayerShared &layer,
                     const LayerConfig &cfg,
                     const LayerIONames &io) {

    impl_->Add(name, layer, cfg, io);
}

void LayerGraph::AddOutput(const std::string &output_name) {

    impl_->AddOutput(output_name);
}

SetS LayerGraph::Outputs() const {

    return impl_->Outputs();
}

SetS LayerGraph::Inputs() const {

    return impl_->Inputs();
}

void LayerGraph::ClearActive() {

    impl_->ClearActive();
}

void LayerGraph::Configure() {

    impl_->Configure();
}

void LayerGraph::Sequence(std::vector<LayerShared> &layer_seq) {

    impl_->Sequence(layer_seq);
}

// template specializations for LayerGraph::Reconfigure()
template <>
int LayerGraph::Reconfigure<bool>(std::string key, const bool& value) {

    return impl_->Reconfigure<bool>(key, value);
}

template <>
int LayerGraph::Reconfigure<float>(std::string key, const float& value) {

    return impl_->Reconfigure<float>(key, value);
}

template <>
int LayerGraph::Reconfigure<int>(std::string key, const int& value) {

    return impl_->Reconfigure<int>(key, value);
}

template <>
int LayerGraph::Reconfigure<std::string>(std::string key, const std::string& value) {

    return impl_->Reconfigure<std::string>(key, value);
}
