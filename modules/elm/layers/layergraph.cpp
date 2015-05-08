/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layergraph.h"

#include "elm/core/debug_utils.h"

#include "elm/core/layerconfig.h"

using namespace std;
using namespace boost;
using namespace elm;

LayerGraph::LayerGraph()
{
    g_ = GraphLayerType(); // no. of vertices unknown yet
}

void LayerGraph::Add(const VtxName &name,
                        std::shared_ptr<base_Layer> &layer,
                        const LayerConfig &cfg,
                        const LayerIONames &io)
{
    // Property accessors
    property_map<GraphLayerType, vertex_name_t>::type
            vtx_name_lut = get(vertex_name, g_);

    VtxDescriptor vtx = add_vertex(name, g_);
    vtx_name_lut[vtx] = name;
}

void LayerGraph::print() {

    cout<<"Vertices"<<endl;

    cout<<num_vertices(g_)<<endl;

    property_map<GraphLayerType, vertex_name_t>::type
            vtx_name_lut = get(vertex_name, g_);

    GraphLayerTraits::vertex_iterator v, end;
    for(tie(v, end) = vertices(g_); v != end; ++v) {

        VtxName key = vtx_name_lut[*v];
        ELM_COUT_VAR(key);
    }
}
