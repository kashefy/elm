/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layergraph.h"

#include <boost/graph/topological_sort.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/boost/ptree_utils_inl.h"
#include "elm/core/exception.h"
#include "elm/core/stl/stl_inl.h"

using namespace std;
using namespace boost;
using namespace elm;

LayerGraph::LayerGraph()
{
}

void LayerGraph::Add(const VtxName &name,
                     const LayerShared &layer_ptr,
                     const LayerConfig &cfg,
                     const LayerIONames &io)
{
    VtxColor color = genVtxColor(name, cfg, io);

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
    layer_vtx.Set(cfg, io, layer_ptr);
    vtx_layer_lut[vtx] = layer_vtx;

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
}

void LayerGraph::ClearActive() {

    property_map<GraphLayerType, vertex_index1_t>::type
            vtx_layer_lut = get(vertex_index1, g_);

    GraphLayerTraits::vertex_iterator v, end;
    for(tie(v, end) = vertices(g_); v != end; ++v) {

        vtx_layer_lut[*v].is_active = false;
    }
}

void LayerGraph::findParents(const VtxDescriptor &child, std::vector<VtxDescriptor> &parents) const
{
    GraphLayerType::inv_adjacency_iterator itr, end;
    for(tie(itr, end) = inv_adjacent_vertices(child, g_);
        itr != end;
        ++itr) {

        VtxDescriptor parent = *itr;
        parents.push_back(parent);
        findParents(parent, parents);
    }
}

void LayerGraph::Sequence(std::vector<LayerShared> &layer_seq) {

    std::vector<VtxDescriptor > q;
    Toposort(q);

    property_map<GraphLayerType, vertex_index1_t>::type
            vtx_layer_lut = get(vertex_index1, g_);

    for (std::vector<VtxDescriptor >::reverse_iterator itr=q.rbegin();
         itr != q.rend();
         ++itr ) {

        if(vtx_layer_lut[*itr].is_active) {

            layer_seq.push_back(vtx_layer_lut[*itr].ptr);
        }
    }
}

void LayerGraph::AddOutput(const std::string &name) {

    property_map<GraphLayerType, vertex_index1_t>::type
            vtx_layer_lut = get(vertex_index1, g_);

    bool found_name = false;
    GraphLayerTraits::vertex_iterator v, end;
    VtxDescriptor vtx_cur;
    for(tie(v, end) = vertices(g_); v != end && !found_name; ++v) {

        VecS vtx_outs = elm::Values(vtx_layer_lut[*v].io.OutputMap());

        VecS::const_iterator itr;
        itr = std::find(vtx_outs.begin(), vtx_outs.end(), name);

        if(itr != vtx_outs.end()) {

            found_name = true;
            vtx_cur = *v;
        }
    } // vertices

    if(!found_name) {

        stringstream s;
        s << "Output (" << name << ") not supported.";
        ELM_THROW_KEY_ERROR(s.str());
    }
    else {

        property_map<GraphLayerType, vertex_name_t>::type
                vtx_name_lut = get(vertex_name, g_);

        std::cout<<"current: "<<vtx_name_lut[vtx_cur]<<std::endl;

        vtx_layer_lut[vtx_cur].is_active = true;

        std::vector<VtxDescriptor> parents;
        this->findParents(vtx_cur, parents);
        for(size_t i=0; i<parents.size(); i++) {

            vtx_layer_lut[parents[i]].is_active = true;
        }

        std::cout<<"parents:";
        for(size_t i=0; i<parents.size(); i++) {

            std::cout<<vtx_name_lut[parents[i]]<<",";
        }
        std::cout<<std::endl;
    }
}

SetS LayerGraph::Outputs() const {

    SetS outputs;

    GraphLayerTraits::vertex_iterator v, end;
    for(tie(v, end) = vertices(g_); v != end; ++v) {

        const LayerWrap &tmp = boost::get(vertex_index1, g_, *v);
        if(tmp.is_active) {

            VecS vtx_outs = elm::Values(tmp.io.OutputMap());
            outputs.insert(vtx_outs.begin(), vtx_outs.end());
        }
    }

    return outputs;
}

void LayerGraph::Toposort(std::vector<VtxDescriptor > &q) {

    q.clear();
    topological_sort(g_, std::back_inserter(q));
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
    std::vector<VtxDescriptor > sort_q;
    Toposort(sort_q);

    for(size_t i=0; i<sort_q.size(); ++i) {


        std::cout<<vtx_name_lut[sort_q[i]]<<std::endl;
    }
}
