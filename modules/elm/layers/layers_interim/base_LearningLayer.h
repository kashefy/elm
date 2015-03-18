/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERS_INTERIM_LEARNINGLAYER_H_
#define _ELM_LAYERS_LAYERS_INTERIM_LEARNINGLAYER_H_

#include "elm/core/base_Layer.h"
#include "elm/core/cv/typedefs_fwd.h"

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
     * Useful for online unsupervised learning (e.g. STDP in WTA circuits)
     */
    virtual void Learn() = 0;

    /**
     * @brief Learn from batch of features and labels
     * Useful for batch supervised learning
     *
     * @param features training data, sample per row
     * @param labels row or column matrix per sample
     */
    virtual void Learn(const cv::Mat1f& features, const cv::Mat1f &labels) = 0;

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

#endif // _ELM_LAYERS_LAYERS_INTERIM_LEARNINGLAYER_H_
