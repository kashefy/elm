#ifndef _ELM_LAYERS_DIFFOFGAUSSIANS_H_
#define _ELM_LAYERS_DIFFOFGAUSSIANS_H_

#include "elm/layers/base_layer_derivations/base_featuretransformationlayer.h"

namespace elm {

class DiffOfGaussians
        : base_FeatureTransformationLayer
{
public:
    DiffOfGaussians();

    DiffOfGaussians(const LayerConfig &cfg);

    void Clear();

    void Reset(const LayerConfig &config);
};

} // namespace elm

#endif // _ELM_LAYERS_DIFFOFGAUSSIANS_H_
