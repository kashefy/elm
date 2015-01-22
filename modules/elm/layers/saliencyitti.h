/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_LAYERS_SALIENCYITTI_H_
#define ELM_LAYERS_SALIENCYITTI_H_

#include <memory>

#include "elm/core/base_Layer.h"
#include "elm/core/sampler.h"
#include "elm/encoding/intensitycontrast.h"
#include "elm/encoding/orientation.h"
#include "elm/encoding/populationcode.h"

/**
 * @brief Implement Itti et al.'s boottom-up saliency measure
 * @todo fix contribution of orientation conspicuty, overshadowed by intensity contrast in saliency map
 */
class SaliencyItti : public base_Layer
{
public:
    static const std::string KEY_INPUT_SCENE;           ///< key to visual scene stimulus
    static const std::string KEY_OUTPUT_SALIENCY;       ///< key to saliency measure
    static const std::string KEY_OUTPUT_SALIENT_LOC;    ///< key to most recently attended location

    static const std::string PARAM_RADIUS;  ///< kernel radius
    static const std::string PARAM_SIGMA;   ///< gabor envelope sigma
    static const std::string PARAM_LAMBDA;  ///< gabor sinusoid wavelength
    static const std::string PARAM_GAMMA;   ///< gabor gamma
    static const std::string PARAM_PS;      ///< gabor phase shift in radians
    static const std::string PARAM_ORIENT_RESPONSE_PERCENTILE; ///< orientation response percentile below which to clip off

    static const int DEFAULT_RADIUS     = 9;    ///< [pixels]
    // remaining default values initialized in source file.
    static const float DEFAULT_SIGMA;//    = 3;
    static const float DEFAULT_LAMBDA;//   = 10;
    static const float DEFAULT_GAMMA;//    = 0.02;
    static const float DEFAULT_PS;// = 0;    ///< [radians]
    static const float DEFAULT_ORIENT_RESPONSE_PERCENTILE;// = 0.7f

    virtual ~SaliencyItti();

    SaliencyItti();

    SaliencyItti(const LayerConfig &config);

    virtual void Clear();

    virtual void Reset(const LayerConfig &config);

    virtual void Reconfigure(const LayerConfig &config);

    virtual void IONames(const LayerIONames &config);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

protected:
    std::string name_scene_;        ///< name of scene in signal
    std::string name_saliency_;     ///< name of saliency map in signal
    std::string name_salient_loc_;  ///< name of salient loc in signal

    RetGang intensity_constrast_;   ///< Intensity contrast measure

    cv::Mat1f saliency_;            ///< saliency map of most recent stimulus

    std::unique_ptr<base_FilterBank> gabors_;   ///< filter bank of orientation-sensitive gabor kernels (V1 simple cells)
    SoftMaxPopulationCode pop_code_orient_; ///< population coding for orientations

    cv::Mat1f theta_range_;         ///< supported orientation angles

    Sampler2D saliency_sampler_;    ///< Sample salient locations

    float percentile_orientation_response_; ///< percentile for masking low orientation responses
};

#endif // ELM_LAYERS_SALIENCYITTI_H_
