/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_SINKHORNBALANCING_H_
#define _ELM_LAYERS_SINKHORNBALANCING_H_

#include <string>

#include <opencv2/core/core.hpp>

#include "elm/layers/base_layer_derivations/base_singleinputfeaturelayer.h"

namespace elm {
/**
 * @brief Layer for implementing Singhorn's balancing algorithm
 * @todo find lighter representation, a full layer is overkill
 * @cites inkhorn1964
 *
 * key to input mat defined in parent
 */
class SinkhornBalancing : public base_SingleInputFeatureLayer
{
public:
    // params
    static const std::string PARAM_EPSILON;     ///< upper excl. convergence threshold
    static const std::string PARAM_MAX_ITER;    ///< max. no. of iterations allowed for Sinkhorn's balancing method

    // I/O Names
    static const std::string KEY_OUTPUT_MAT_BALANCED;   ///< key to output mat
    static const std::string KEY_OUTPUT_IS_CONVERGED;   ///< key to convergence result

    SinkhornBalancing();

    SinkhornBalancing(const LayerConfig &cfg);

    virtual void Clear();

    virtual void Reset(const LayerConfig &config);

    virtual void Reconfigure(const LayerConfig &config);

    virtual void OutputNames(const LayerOutputNames &io);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

    /**
     * @brief Apply Sinkhorn's balancing algroithm for iterative row-col normalization
     * @param m_ai0 modified in-place
     * @return true on convergence
     */
    static bool RowColNormalization(cv::Mat1f &m, int max_iter, float epsilon);

protected:
    std::string name_out_m_;    ///< destination name for output
    std::string name_out_convergence_;    ///< destination name for convergence result

    cv::Mat1f m_;

    bool is_converged;  ///< convergence result

    float epsilon_; ///< @see PARAM_EPSILON
    int max_iter_;  ///< @see PARAM_MAX_ITER
};

} // namespace elm

#endif // _ELM_LAYERS_SINKHORNBALANCING_H_
