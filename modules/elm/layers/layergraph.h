/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERGRAPH_H_
#define _ELM_LAYERS_LAYERGRAPH_H_

#include <string>

#include "elm/core/typedefs_fwd.h"

namespace elm {

class Signal;
class LayerGraph_Impl;

class LayerGraph
{
public:
    ~LayerGraph();

    LayerGraph();

    void Add(LayerShared &layer, const LayerConfig &cfg, const LayerIONames &io);

    void AddOutput(const std::string &output_name);

    void RemoveOutput(const std::string &output_name);

    bool HasInputs(const Signal &s) const;

protected:
    LayerGraph_Impl *impl_;
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERGRAPH_H_
