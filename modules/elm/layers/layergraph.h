/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERLGRAPH_H_
#define _ELM_LAYERS_LAYERLGRAPH_H_

#include <memory>
#include <string>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "elm/core/base_Layer.h"
#include "elm/core/layerconfig.h"

typedef std::string EdgeName;
typedef boost::property<boost::edge_name_t, EdgeName> EdgeProp;

typedef std::shared_ptr<elm::base_Layer > LayerShared;
typedef std::string VtxColor;
typedef std::string VtxName;
typedef boost::property<boost::vertex_color_t, VtxColor,
            boost::property<boost::vertex_name_t, VtxName,
                boost::property<boost::vertex_index1_t, elm::LayerConfig,
                    boost::property<boost::vertex_degree_t, elm::LayerIONames,
                        boost::property<boost::vertex_index2_t, LayerShared> // layer
                    > // io
                > // config
            > // name
        > // color
        VtxProp;

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VtxProp, EdgeProp> GraphLayerType;

typedef boost::graph_traits<GraphLayerType> GraphLayerTraits;
typedef GraphLayerTraits::edge_iterator edge_iter;
typedef GraphLayerTraits::vertex_descriptor VtxDescriptor;

namespace std {

template <typename T> class shared_ptr; ///< convinience typedef for point template

} // namespace std

namespace elm {

class Signal;

class LayerGraph
{
public:
    LayerGraph();

    void Add(const VtxName &name,
             std::shared_ptr<base_Layer> &layer,
             const LayerConfig &cfg,
             const LayerIONames &io);

    void AddOutput(const std::string &output_name);

    void RemoveOutput(const std::string &output_name);

    bool HasInputs(const Signal &s) const;

    VtxColor genVtxColor(const VtxName &name,
                         const LayerConfig &cfg,
                         const LayerIONames &io) const;

    void print();

protected:
    GraphLayerType g_;  ///< graph member
    std::set<std::string> outputs_;
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERLGRAPH_H_
