/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERLGRAPH_IMPL_H_
#define _ELM_LAYERS_LAYERLGRAPH_IMPL_H_

#include <memory>
#include <string>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "elm/core/base_Layer.h"
#include "elm/core/layerconfig.h"
#include "elm/core/typedefs_fwd.h"

#include "elm/core/boost/ptree_utils_inl.h"

namespace elm {

/** @brief wrap layer information to store as vertex
 */
struct LayerVertex {

    LayerVertex()
        : is_active(false)
    {}

    /**
     * @brief Set necessary info
     * @param _cfg configuration
     * @param _io I/O names
     * @param _ptr pointer to instance
     */
    void Set(const LayerConfig &_cfg,
             const LayerIONames &_io,
             const LayerShared &_ptr) {

        cfg = _cfg;
        io  = _io;
        ptr = _ptr;
    }

    void Configure() {

        ptr->Reset(cfg);
        ptr->IONames(io);
    }

    LayerConfig cfg;    ///< configuration
    LayerIONames io;    ///< I/O
    LayerShared ptr;    ///< pointer to shared instance
    bool is_active;     ///< flag if active or not
};

} // namespace elm

typedef std::string EdgeName;
typedef boost::property<boost::edge_name_t, EdgeName> EdgeProp;

typedef std::shared_ptr<elm::base_Layer > LayerShared;
typedef std::string VtxColor;
typedef std::string VtxName;
typedef boost::property<boost::vertex_color_t, VtxColor,
            boost::property<boost::vertex_name_t, VtxName,
                boost::property<boost::vertex_index1_t, elm::LayerVertex,
                    boost::property<boost::vertex_index2_t, int> // idx (order)
                > // layer
            > // name
        > // color
        VtxProp;

typedef boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::bidirectionalS,
    VtxProp, EdgeProp> GraphLayerType;

typedef boost::graph_traits<GraphLayerType> GraphLayerTraits;
typedef GraphLayerTraits::edge_iterator edge_iter;
typedef GraphLayerTraits::vertex_descriptor VtxDescriptor;

namespace elm {

class Signal;
typedef std::set<std::string> SetS;     ///< convinience typedef for set of strings

/** @brief Layer Graph implentation for managing layer pipelines
  * credit: J. Turcot
  */
class LayerGraph_Impl
{
public:
    LayerGraph_Impl();

    /**
     * @brief Add layer info as vertex to graph
     * @param name
     * @param layer shared layer instance
     * @param cfg configuration
     * @param io I/O
     */
    void Add(const VtxName &name,
             const LayerShared &layer,
             const LayerConfig &cfg,
             const LayerIONames &io);

    /**
     * @brief Clear Active layers
     */
    void ClearActive();

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
     * @brief Generate unique id for vertex
     * @param name layer name
     * @param cfg layer configuration
     * @param io layer I/O settings
     * @return vertex color
     */
    VtxColor genVtxColor(const VtxName &name,
                         const LayerConfig &cfg,
                         const LayerIONames &io) const;

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
    int Reconfigure(const std::string &key, const TVal& value);

    /**
     * @brief Get Sequence of layers for generating requested outputs
     * @param[out] ordered list of layers
     */
    void Sequence(std::vector<LayerShared> &layer_seq);

    //void print();

protected:
    /**
     * @brief find vertex parents recursively
     * @param[in] child vertex
     * @param[out] parents immediate first
     */
    void findParents(const VtxDescriptor &child, std::vector<VtxDescriptor> &findParents) const;

    /**
     * @brief Perform topological sorting of layer graph
     * @param[out] q vertices in reverse topological order
     */
    void Toposort(std::vector<VtxDescriptor > &q);

    GraphLayerType g_;  ///< graph member
};

} // namespace elm

namespace elm {

template <class TVal>
int LayerGraph_Impl::Reconfigure(const std::string &key, const TVal& value) {

    int count = 0;

    boost::property_map<GraphLayerType, boost::vertex_index1_t>::type
            vtx_layer_lut = get(boost::vertex_index1, g_);

    GraphLayerTraits::vertex_iterator v, end;
    for(boost::tie(v, end) = vertices(g_); v != end; ++v) {

        LayerConfig cfg = vtx_layer_lut[*v].cfg;

        PTree p = cfg.Params();
        //PrintXML(p, std::cout);std::cout<<std::endl;
        PTree::assoc_iterator tmp_itr = p.find(key);

        if(tmp_itr != p.not_found()) {

            PTree::iterator itr = p.to_iterator(tmp_itr);
            itr->second.put_value<TVal>(value);

            //PrintXML(p, std::cout);std::cout<<std::endl;

            cfg.Params(p); // update layer config

            vtx_layer_lut[*v].cfg = cfg; // update configuration object
            vtx_layer_lut[*v].ptr->Reconfigure(cfg); // reconfigure layer

            count++;
        }
    } // vertices

    return count;
}

} // namespace elm

#endif // _ELM_LAYERS_LAYERLGRAPH_IMPL_H_
