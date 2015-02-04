/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_GRAPHCOMPATIBILITY_H_
#define _ELM_LAYERS_GRAPHCOMPATIBILITY_H_

#include "elm/layers/base_layer_derivations/base_matoutputlayer.h"

namespace elm {

class GraphCompatibility : public base_MatOutputLayer
{
public:
    // I/O Names
    static const std::string KEY_INPUT_GRAPH_AB;    ///< key to graph ab's adjacency matrix
    static const std::string KEY_INPUT_GRAPH_IJ;    ///< key to graph ij's adjacency matrix
    // output key defined in parent layer

    virtual ~GraphCompatibility();

    GraphCompatibility();

    GraphCompatibility(const LayerConfig &cfg);

    virtual void Clear();

    virtual void Reset(const LayerConfig &config);

    virtual void Reconfigure(const LayerConfig &config);

    virtual void InputNames(const LayerIONames &io);

    virtual void Activate(const Signal &signal);

protected:
    /**
     * @brief calculate compatibility matrix C_aibj Eq. (2) from \cite Gold1996
     *
     * Assuming symmetrical graph adjacency matrices
     *
     * @param g_ab adjacency matrix
     * @param g_ij adjacency matrix
     * @return compatibility matrix
     */
    virtual cv::Mat1f Compatibility(const cv::Mat1f &g_ab, const cv::Mat1f &g_ij) const;

    /**
     * @brief compatibility function for populating compatibility matrix C_aibj Eq. (2) from \cite Gold1996
     *
     * if either wieght is "NULL" c=0, otherwise 1-3*|w1-w2|
     *
     * @param w1 weight form frist graph
     * @param w2 weight from second graph
     * @return compatibility score
     */
    virtual float Compatibility(float w1, float w2) const;

    std::string name_g_ab_;     ///< name of graph ab adj. matrix in signal object
    std::string name_g_ij_;     ///< name of graph ij adj. matrix in signal object

    int A_;                 /// no. of vertices in graph g_ab
    int I_;                 /// no. of vertices in graph g_ij
};

} // namespace elm

#endif // _ELM_LAYERS_GRAPHCOMPATIBILITY_H_
