#ifndef SEM_LAYERS_SALIENCYITTI_H_
#define SEM_LAYERS_SALIENCYITTI_H_

#include "core/base_Layer.h"
#include "core/sampler.h"
#include "encoding/intensitycontrast.h"
#include "encoding/orientation.h"
#include "encoding/populationcode.h"

/**
 * @brief Implement Itti et al.'s boottom-up saliency measure
 */
class SaliencyItti : public base_Layer
{
public:
    static const std::string KEY_INPUT_SCENE;           ///< key to visual scene stimulus
    static const std::string KEY_OUTPUT_SALIENCY;       ///< key to saliency measure
    static const std::string KEY_OUTPUT_SALIENT_LOC;    ///< key to most recently attended location

    virtual ~SaliencyItti();

    SaliencyItti();

    virtual void Reset();

    virtual void Reconfigure(const LayerConfig &config);

    virtual void Stimulus(const Signal &signal);

    virtual void Apply();

    virtual void Response(Signal &signal);

protected:
    std::string name_scene_;        ///< name of scene in signal
    std::string name_saliency_;     ///< name of saliency map in signal
    std::string name_salient_loc_;  ///< name of salient loc in signal

    RetGang intensity_constrast_;   ///< Intensity contrast measure

    cv::Mat1f saliency_;            ///< saliency map of most recent stimulus
    cv::Mat1f stimulus_;            ///< single channel/grayscale stimulus

    VecMat1f kernels_orient_;               ///< orientation-sensitive kernels (V1 simple cells)
    SoftMaxPopulationCode pop_code_orient_; ///< population coding for orientations

    cv::Mat1f theta_range_;         ///< supported orientation angles

    Sampler2D saliency_sampler_;    ///< Sample salient locations
};

#endif // SEM_LAYERS_SALIENCYITTI_H_
