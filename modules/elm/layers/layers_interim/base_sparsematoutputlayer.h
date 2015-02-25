/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERS_INTERIM_BASE_SPARSEMATOUTPUTLAYER_H
#define _ELM_LAYERS_LAYERS_INTERIM_BASE_SPARSEMATOUTPUTLAYER_H

#include <string>

#include <opencv2/core/core.hpp>

#include "elm/core/typedefs_fwd.h"
#include "elm/core/base_Layer.h"

namespace elm {

class Signal;

namespace detail {

const std::string BASE_SPARSEMATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE = "m";   ///< define string here to ensure early initialization for test purposes

}

/**
 * @brief intermediate layer class assuming single sparse matrix output
 * @todo generalize to FeatureOutputLayer, or maybe keep this (compiles faster)
 */
class base_SparseMatOutputLayer : virtual public base_Layer
{
public:
    static const std::string KEY_OUTPUT_RESPONSE;

    virtual ~base_SparseMatOutputLayer();

    virtual void OutputNames(const LayerOutputNames &io);

    virtual void Response(Signal &signal);

protected:
    base_SparseMatOutputLayer();

    base_SparseMatOutputLayer(const LayerConfig& cfg);

    // members
    std::string name_output_;    ///< destination name in signal object

    elm::SparseMat1f m_;   ///< most recent response/output
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERS_INTERIM_BASE_SPARSEMATOUTPUTLAYER_H
