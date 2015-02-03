/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BASE_LAYER_DERIVS_BASE_MATINPUTLAYER_H_
#define _ELM_CORE_BASE_LAYER_DERIVS_BASE_MATINPUTLAYER_H_

#include <string>

#include "elm/core/base_Layer.h"

namespace elm {

class Signal;

namespace detail {

const std::string BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT = "input";   ///< define string here to ensure early initialization for test purposes

}

/**
 * @brief base class for layer assuming single feature input, an intermediate layer implementation
 */
class base_SingleInputFeatureLayer : virtual public base_Layer
{
public:
    static const std::string KEY_INPUT;

    virtual ~base_SingleInputFeatureLayer();

    /**
     * @brief sets name_input_ with feature name
     * @param io
     */
    virtual void IONames(const LayerIONames &io);

protected:
    base_SingleInputFeatureLayer();

    base_SingleInputFeatureLayer(const LayerConfig& cfg);

    // members
    std::string name_input_;    ///< name to input feature in signal object
};

} // namespace elm

#endif // _ELM_CORE_BASE_LAYER_DERIVS_BASE_MATINPUTLAYER_H_
