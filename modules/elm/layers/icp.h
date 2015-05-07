/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_ICP_H_
#define _ELM_LAYERS_ICP_H_

#include <string>

#ifndef __WITH_PCL
    // Disabling ICP layer due to no PCL support and defining it as a non-supported
    #include "elm/layers/layernotsupported.h"
namespace elm {
    class ICP : public ELM_LAYER_NOT_SUPPORTED(ICP, "Building with PCL is required for supporting ICP layer.");
} // namespace elm
#else   // __WITH_PCL

#include <opencv2/core/core.hpp>

#include "elm/core/base_Layer.h"

namespace elm {

/**
 * @brief class for an iterative-closest-point layer
 * This is basciallly a wrapper around PCL's ICP implementation
 *
 * Requires building with PCL support
 *
 * @todo expose paramters
 */
class ICP : public base_Layer
{
public:
    // I/O names
    static const std::string KEY_INPUT_POINT_CLOUD_SRC;         ///< key to source cloud
    static const std::string KEY_INPUT_POINT_CLOUD_TARGET;      ///< key to target cloud

    static const std::string KEY_OUTPUT_SCORE;                  ///< key to fitness score
    static const std::string KEY_OUTPUT_CONVERGENCE;            ///< key to convergence result
    static const std::string KEY_OUTPUT_TRANSFORMATION;         ///< key to optional final tansformation

    ICP();

    void Clear();

    void Reconfigure(const LayerConfig &cfg);

    void InputNames(const LayerInputNames &config);

    void OutputNames(const LayerOutputNames &config);

    void Activate(const Signal &signal);

    void Response(Signal &signal);

protected:
    std::string name_src_cloud_;        ///< name of input point cloud in signal object
    std::string name_target_cloud_;     ///< name of input target cloud in signal object
    std::string name_score_;            ///< destination name of fitness score in signal object
    std::string name_convergence_;      ///< destination name of convergence results
    std::string name_transf_;           ///< desitnation name of final transformation

    bool has_conveged_;                 ///< flag whether icp converged
    float fitness_score_;               ///< fitness score to most recent stimuli
    cv::Mat1f transformation_;          ///< final estimated transformation matrix
};

} // namespace elm

#endif  // __WITH_PCL

#endif // _ELM_LAYERS_ICP_H_
