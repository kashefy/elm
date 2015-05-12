/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layergraph.h"

#include <boost/graph/topological_sort.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/boost/ptree_utils_inl.h"
#include "elm/core/stl/stl_inl.h"

using namespace std;
using namespace boost;
using namespace elm;

LayerGraph::LayerGraph()
    : count_(0)
{
}

void LayerGraph::Add(const VtxName &name,
                        std::shared_ptr<base_Layer> &layer_ptr,
                        const LayerConfig &cfg,
                        const LayerIONames &io)
{
    VtxColor color = genVtxColor(name, cfg, io);

//    bool is_new_vtx;
    //VtxDescriptor vtx = retrieveVertex(value_cur, is_new_vtx);

    VtxDescriptor vtx = add_vertex(color, g_);

    // Property accessors
    property_map<GraphLayerType, vertex_color_t>::type
            vtx_color_lut = get(vertex_color, g_);
    vtx_color_lut[vtx] = color;

    property_map<GraphLayerType, vertex_name_t>::type
            vtx_name_lut = get(vertex_name, g_);
    vtx_name_lut[vtx] = name;

    property_map<GraphLayerType, vertex_index1_t>::type
            vtx_layer_lut = get(vertex_index1, g_);

    LayerWrap layer_vtx;
    layer_vtx.cfg = cfg;
    layer_vtx.io = io;
    layer_vtx.ptr = layer_ptr;
    vtx_layer_lut[vtx] = layer_vtx;

    property_map<GraphLayerType, vertex_index2_t>::type
            vtx_idx_lut = get(vertex_index2, g_);

    active_.push_back(count_);
    vtx_idx_lut[vtx] = count_;

    //
    // inputs
    VecS inputs = elm::Values(io.InputMap());
    VecS outputs = elm::Values(io.OutputMap());

    // Iterate through the edges
//    property_map<GraphLayerType, edge_name_t >::type
//        edge_name_lut = get(edge_name, g_);

//    edge_iter ei, ei_end;
//    for (tie(ei, ei_end) = edges(g_); ei != ei_end; ++ei) {

//        EdgeName edge_name = edge_name_lut[*ei];
//        EdgeProp edge_prop = edge_name;

//        VecS::const_iterator itr;

//        itr = std::find(inputs.begin(), inputs.end(), edge_name);

//        if(itr != inputs.end()) {

//            VtxDescriptor src = source(*ei, g_);
//            add_edge(src, vtx, edge_prop, g_);
//        }

//        itr = std::find(outputs.begin(), outputs.end(), edge_name);

//        if(itr != outputs.end()) {

//            VtxDescriptor src = target(*ei, g_);
//            add_edge(vtx, src, edge_prop, g_);
//        }
//    }


    for (auto const& input : inputs) {

        GraphLayerTraits::vertex_iterator v, end;
        for(tie(v, end) = vertices(g_); v != end; ++v) {

            LayerIONames vtx_io = vtx_layer_lut[*v].io;
            VecS vtx_outs = elm::Values(vtx_io.OutputMap());

            VecS::const_iterator itr;
            itr = std::find(vtx_outs.begin(), vtx_outs.end(), input);

            if(itr != vtx_outs.end()) {

                EdgeProp edge_prop = *itr;

                add_edge(*v, vtx, edge_prop, g_);
            }
        } // vertices
    } // inputs

    for (auto const& output : outputs) {

        GraphLayerTraits::vertex_iterator v, end;
        for(tie(v, end) = vertices(g_); v != end; ++v) {

            LayerIONames vtx_io = vtx_layer_lut[*v].io;
            VecS vtx_ins = elm::Values(vtx_io.InputMap());

            VecS::const_iterator itr;
            itr = std::find(vtx_ins.begin(), vtx_ins.end(), output);

            if(itr != vtx_ins.end()) {

                EdgeProp edge_prop = *itr;
                add_edge(vtx, *v, edge_prop, g_);
            }
        } // vertices
    } // outputs

    count_++;
}

void LayerGraph::AddOutput(const std::string &output_name) {

    outputs_.insert(output_name);
}

void LayerGraph::RemoveOutput(const std::string &output_name) {

    outputs_.erase(output_name);
}

void LayerGraph::Toposort() {

    std::vector<VtxDescriptor > q;
    topological_sort(g_, std::back_inserter(q));

    property_map<GraphLayerType, vertex_name_t>::type
            vtx_name_lut = get(vertex_name, g_);

    for(size_t i=0; i<q.size(); ++i) {


        std::cout<<vtx_name_lut[q[i]]<<std::endl;
    }
}

VtxColor LayerGraph::genVtxColor(const VtxName &name,
                     const LayerConfig &cfg,
                     const LayerIONames &io) const {


    VtxColor color = name;

    stringstream s;
    PrintXML(cfg.Params(), s);

    color += s.str();

    // inputs
    MapSS io_map = io.InputMap();
    color += elm::to_string(elm::Keys(io_map));
    color += elm::to_string(elm::Values(io_map));

    // outputs
    io_map = io.OutputMap();
    color += elm::to_string(elm::Keys(io_map));
    color += elm::to_string(elm::Values(io_map));

    return color;
}

void LayerGraph::print() {

    cout<<"Vertices:"<<endl;

    cout<<num_vertices(g_)<<endl;

    property_map<GraphLayerType, vertex_name_t>::type
            vtx_name_lut = get(vertex_name, g_);

    GraphLayerTraits::vertex_iterator v, end;
    for(tie(v, end) = vertices(g_); v != end; ++v) {

        VtxName key = vtx_name_lut[*v];
        ELM_COUT_VAR(key);
    }

    cout<<"Edges:"<<endl;

    property_map<GraphLayerType, edge_name_t >::type
        edge_name_lut = get(edge_name, g_);
    edge_iter ei, ei_end;
    for (tie(ei, ei_end) = edges(g_); ei != ei_end; ++ei) {

        EdgeName edge_name = edge_name_lut[*ei];

        VtxDescriptor src = source(*ei, g_);
        VtxName src_name = vtx_name_lut[src];

        VtxDescriptor dst = target(*ei, g_);
        VtxName dst_name = vtx_name_lut[dst];

        cout<<src_name<<"->"<<edge_name<<"->"<<dst_name<<endl;
    }

    std::cout<<"Toposort..."<<std::endl;
    Toposort();
}
