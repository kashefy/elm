/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BASE_LAYER_DERIVS_BASE_TIVLAYER_H_
#define _ELM_CORE_BASE_LAYER_DERIVS_BASE_TIVLAYER_H_

#include "elm/core/base_Layer.h"

namespace elm {

/**
 * @brief class for time-invariant layer
 */
class base_TIVLayer : public base_Layer
{
public:
    virtual ~base_TIVLayer();

protected:
    base_TIVLayer();

    base_TIVLayer(const LayerConfig& cfg);
};

} // namespace elm

#endif // _ELM_CORE_BASE_LAYER_DERIVS_BASE_TIVLAYER_H_
