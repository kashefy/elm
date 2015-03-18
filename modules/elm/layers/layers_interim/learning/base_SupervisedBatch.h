/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERS_INTERIM_LEARNING_BATCHSUPERVISED_H_
#define _ELM_LAYERS_LAYERS_INTERIM_LEARNING_BATCHSUPERVISED_H_

#include "elm/layers/layers_interim/base_LearningLayer.h"
#include "elm/core/cv/typedefs_fwd.h"

namespace elm {

class LayerConfig;

/**
 * @brief class for defining interfaces of an abstract layer that can learn
 * Overloaded constructor calls overloaded Reset() so you can implement Reset(config) directly.
 */
class base_SupervisedBatch : public elm::base_LearningLayer
{
public:
    virtual ~base_SupervisedBatch();

    /** @throws elm::ExceptionNotImpl
     */
    void Learn();

    virtual void Learn(const cv::Mat1f& features, const cv::Mat1f &labels) = 0;

protected:
    base_SupervisedBatch();

    /**
     * @brief Construct and fully configure a layer (parameters and IO)
     * Derived class should call Reset(config) and IO(config) from here.
     * @param config
     */
    base_SupervisedBatch(const LayerConfig& config);
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERS_INTERIM_LEARNING_BATCHSUPERVISED_H_
