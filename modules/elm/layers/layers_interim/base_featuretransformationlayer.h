/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERS_INTERIM_BASE_FEATURETRANSFORMATIONLAYER_H_
#define _ELM_LAYERS_LAYERS_INTERIM_BASE_FEATURETRANSFORMATIONLAYER_H_

#include "elm/layers/layers_interim/base_singleinputfeaturelayer.h"
#include "elm/layers/layers_interim/base_matoutputlayer.h"

namespace elm {

/**
 * @brief An intermediate base class for prviding a layer interface involving
 * a transformation of a single input feature into a single output feature
 * "1-1 stimulus to response transformation"
 *
 * I/O names (1 for each) are relayed from parent classes
 */
class base_FeatureTransformationLayer:
        public base_SingleInputFeatureLayer,
        public base_MatOutputLayer
{
public:

protected:
    virtual ~base_FeatureTransformationLayer();

    base_FeatureTransformationLayer();
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERS_INTERIM_BASE_FEATURETRANSFORMATIONLAYER_H_
