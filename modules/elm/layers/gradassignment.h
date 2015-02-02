/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_GRADASSIGNMENT_H_
#define _ELM_LAYERS_GRADASSIGNMENT_H_

#include <string>

#include <opencv2/core.hpp>

#include "elm/core/base_Layer.h"

class LayerConfig;
class Signal;

namespace elm {

/**
 * @brief Layer for implementing Graduated Assignment algorithm for graph matching
 *
 * Assuming symmetrical graph adjacency matrices
 *
 * @cite Gold1996
 */
class GradAssignment : public base_Layer
{
public:
    // params
    static const std::string PARAM_BETA;        ///< initial value of the control parameter (deterministic annealing)
    static const std::string PARAM_BETA_MAX;    ///< max. value of the control parameter
    static const std::string PARAM_BETA_RATE;   ///< rate at which control param. increases
    static const std::string PARAM_MAX_ITER_PER_BETA; ///< max. no. of iterations allowed per beta value
    static const std::string PARAM_MAX_ITER_SINKHORN; ///< max. no. of iterations allowed for Sinkhorn's balancing method

    // I/O Names
    static const std::string KEY_INPUT_GRAPH_AB; ///< key to graph ab's adjacency matrix
    static const std::string KEY_INPUT_GRAPH_IJ; ///< key to graph ij's adjacency matrix
    static const std::string KEY_OUTPUT_M;       ///< match matrix variables

    virtual ~GradAssignment();

    GradAssignment();

    GradAssignment(const LayerConfig &cfg);

    virtual void Clear();

    virtual void Reset(const LayerConfig &config);

    virtual void Reconfigure(const LayerConfig &config);

    virtual void IONames(const LayerIONames &config);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

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

    static const float EPSILON; ///< small no. epsilon

    std::string name_g_ab_;     ///< name of graph ab adj. matrix in signal object
    std::string name_g_ij_;     ///< name of graph ij adj. matrix in signal object
    std::string name_out_m_;    ///< desintation name for match matrix

    float beta_0_;      ///< initial value for control param. beta
    float beta_max_;    ///< max. value for control param.
    float beta_rate_;   ///< rate at which control param. increases

    int max_iter_per_beta_; ///< max. no. of iterations allowed per beta value
    int max_iter_sinkhorn_; ///< max. no. of iterations allowed for Sinkhorn's balancing method

    int A_;                 /// no. of vertices in graph g_ab
    int I_;                 /// no. of vertices in graph g_ij

    cv::Mat1f m_ai_;       ///< match matrix variables
};

} // namespace elm

#endif // _ELM_LAYERS_GRADASSIGNMENT_H_
