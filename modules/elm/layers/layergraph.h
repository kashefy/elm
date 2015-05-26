﻿/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERGRAPH_H_
#define _ELM_LAYERS_LAYERGRAPH_H_

#include <string>
#include <set>
#include <vector>

#include "elm/core/typedefs_fwd.h"

namespace elm {

class Signal;
class LayerGraph_Impl;
typedef std::set<std::string> SetS;     ///< convinience typedef for set of strings

/** @brief Layer Graph for managing non-linear layer pipelines
  */
class LayerGraph
{
public:
    ~LayerGraph();

    LayerGraph();

    /**
     * @brief Add layer and associated info to graph
     * @param name layer name
     * @param layer shared layer instance
     * @param cfg configuration
     * @param io I/O
     */
    void Add(const std::string &name,
             LayerShared &layer,
             const LayerConfig &cfg,
             const LayerIONames &io);

    /**
     * @brief Request output
     * @param output_name
     * @throws ExceptionKeyError if unavailable
     */
    void AddOutput(const std::string &output_name);

    /**
     * @brief Get list of outputs generated by graph
     * @return list of unique outputs
     */
    SetS Outputs() const;

    /**
     * @brief Get list of primary inputs required by active layers
     * Primary inputs are inputs not generated by any active layers in the graph
     * @return list of unique primary input names
     */
    SetS Inputs() const;

    /**
     * @brief Clear Active layers
     */
    void ClearActive();

    /**
     * @brief Configure active layers
     */
    void Configure();

    /**
     * @brief reconfigure any layers with configured with given parameter key value pair
     * @param key parameter key
     * @param value new parameter value
     * @return no. of layers updated with new value
     */
    template <class TVal>
    int Reconfigure(std::string key, const TVal& value);

    /**
     * @brief Get Sequence of layers for generating requested outputs
     * @param[out] ordered list of layers
     */
    void Sequence(std::vector<LayerShared> &layer_seq);

protected:
    LayerGraph_Impl *impl_;
};

} // namespace elm

namespace elm {

// template specializations for LayerGraph::Reconfigure()
template <>
int LayerGraph::Reconfigure<bool>(std::string key, const bool& value);

template <>
int LayerGraph::Reconfigure<float>(std::string key, const float& value);

template <>
int LayerGraph::Reconfigure<int>(std::string key, const int& value);

template <>
int LayerGraph::Reconfigure<std::string>(std::string key, const std::string& value);

} // namespace elm

#endif // _ELM_LAYERS_LAYERGRAPH_H_
