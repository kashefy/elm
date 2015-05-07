/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_MLP_H_
#define _ELM_LAYERS_MLP_H_

#include "elm/layers/layers_interim/learning/base_SupervisedBatch.h"

#include <opencv2/ml/ml.hpp>

namespace elm {

/**
 * @brief Layer class for mult-layer perceptrons
 */
class MLP : public base_SupervisedBatch
{
public:
    static const std::string PARAM_ARCH;                ///< network architecture (how many nodes per layer input,[hidden1, hidden2,...], output)
    static const std::string PARAM_TERM_CRITERIA;       ///< termination criteria
    static const std::string PARAM_TRAIN_METHOD;        ///< training method
    static const std::string PARAM_BP_DW_SCALE;         ///< backpropagation(BP)'s delta weight scale
    static const std::string PARAM_BP_MOMENT_SCALE;     ///< BP's moment scale

    virtual ~MLP();

    MLP();

    void Clear();

    void Reset(const LayerConfig &config);

    void Reconfigure(const LayerConfig &config);

    void Activate(const Signal &signal);

    virtual void Learn(const cv::Mat1f &features, const cv::Mat1f &labels);

protected:

    // members
    CvANN_MLP mlp_;                 ///< neural network
    CvANN_MLP_TrainParams params_;  ///< training params
};

} // namespace elm

#endif // _ELM_LAYERS_MLP_H_
