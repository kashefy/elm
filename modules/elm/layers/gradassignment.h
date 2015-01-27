#ifndef ELM_LAYERS_GRADASSIGNMENT_H_
#define ELM_LAYERS_GRADASSIGNMENT_H_

#include <string>

#include <opencv2/core.hpp>

#include "elm/core/base_Layer.h"

class LayerConfig;
class Signal;

namespace elm {

/**
 * @brief Layer for implementing Graduated Assignment algorithm for graph matching
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
    std::string name_g_ab_;     ///< name of graph ab adj. matrix in signal object
    std::string name_g_ij_;     ///< name of graph ij adj. matrix in signal object
    std::string name_out_m_;    ///< desintation name for match matrix

    float beta_0_;      ///< initial value for control param. beta
    float beta_max_;    ///< max. value for control param.
    float beta_rate_;   ///< rate at which control param. increases

    int max_iter_per_beta_; ///< max. no. of iterations allowed per beta value
    int max_iter_sinkhorn_; ///< max. no. of iterations allowed for Sinkhorn's balancing method

    cv::Mat1f m_;       ///< match matrix variables
};

} // namespace elm

#endif // ELM_LAYERS_GRADASSIGNMENT_H_
