/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERLISTIMPL_H_
#define _ELM_LAYERS_LAYERLISTIMPL_H_

#include <string>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

typedef float EdgeWeight;
typedef boost::property<boost::edge_weight_t, EdgeWeight> EdgeWeightProp;

typedef int VtxColor;
typedef std::string VtxIdx2;
typedef boost::property<boost::vertex_color_t, VtxColor,
        boost::property<boost::vertex_index2_t, VtxIdx2 > >
        VtxProp;

typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS, VtxProp, EdgeWeightProp> GraphAttrType;

typedef boost::graph_traits<GraphAttrType> GraphAttrTraits;
typedef GraphAttrTraits::edge_iterator edge_iter;
typedef GraphAttrTraits::vertex_descriptor VtxDescriptor;

namespace std {

template <typename T> class shared_ptr; ///< convinience typedef for point template

} // namespace std

namespace elm {

class base_Layer;
class LayerConfig;
class LayerIONames;
class Signal;

class LayerListImpl
{
public:
    LayerListImpl();

    void Add(std::shared_ptr<base_Layer> &layer, const LayerConfig &cfg, const LayerIONames &io);

    void AddOutput(const std::string &output_name);

    void RemoveOutput(const std::string &output_name);

    bool HasInputs(const Signal &s) const;

protected:
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERLISTIMPL_H_
