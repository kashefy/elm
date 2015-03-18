#ifndef _ELM_LAYERS_MLP_H_
#define _ELM_LAYERS_MLP_H_

#include "elm/layers/layers_interim/learning/base_SupervisedBatch.h"

namespace elm {

/**
 * @brief Layer class for mult-layer perceptrons
 */
class MLP : public base_SupervisedBatch
{
public:
    ~MLP();

    MLP();

    MLP(const LayerConfig &cfg);

    void Clear();

    void Reset(const LayerConfig &config);

    void Reconfigure(const LayerConfig &config);

    void Activate(const Signal &signal);

    virtual void Learn(const cv::Mat1f &features, const cv::Mat1f &labels);
};

} // namespace elm

#endif // _ELM_LAYERS_MLP_H_
