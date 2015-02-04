/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BASE_LAYER_DERIVS_LEARNINGLAYER_H_
#define _ELM_CORE_BASE_LAYER_DERIVS_LEARNINGLAYER_H_

#include "elm/core/base_Layer.h"

namespace elm {

class LayerConfig;

/**
 * @brief class for defining interfaces of an abstract layer that can learn
 * Overloaded constructor calls overloaded Reset() so you can implement Reset(config) directly.
 */
class base_LearningLayer : public base_Layer
{
public:
    virtual ~base_LearningLayer();

    /**
     * @brief Learn from most recent stimuli
     */
    virtual void Learn() = 0;

protected:
    base_LearningLayer();

    /**
     * @brief Construct and fully configure a layer (parameters and IO)
     * Derived class shoudl call Reset(config) and IO(config) from here.
     * @param config
     */
    base_LearningLayer(const LayerConfig& config);
};

} // namespace elm

#endif // _ELM_CORE_BASE_LAYER_DERIVS_LEARNINGLAYER_H_
