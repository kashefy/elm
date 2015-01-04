#ifndef SEM_LAYERS_ICP_H_
#define SEM_LAYERS_ICP_H_

#include <string>

#include "core/base_Layer.h"

/**
 * @brief class for an iterative-closest-point layer
 * Effectively wraps around PCL's ICP implementation
 */
class ICP : public base_Layer
{
public:
    // I/O names
    static const std::string KEY_INPUT_POINT_CLOUD_SRC;         ///< key to source cloud
    static const std::string KEY_INPUT_POINT_CLOUD_TARGET;      ///< key to target cloud
    static const std::string KEY_OUTPUT_SCORE;                  ///< key to fitness score
    static const std::string KEY_OUTPUT_CONVERGENCE;            ///< key to convergence result
    static const std::string KEY_OUTPUT_TRANSFORMATION;     ///< key to optional final tansformation

    ICP();

    ICP(const LayerConfig &cfg);

    void Clear();

    void Reset();

    void Reset(const LayerConfig &cfg);

    void IONames(const LayerIONames &config);

    void Activate(const Signal &signal);

    void Response(Signal &signal);

protected:
    std::string name_src_cloud_;     ///< name of input point cloud in signal object
    std::string name_target_cloud_;  ///< name of input target cloud in signal object
    std::string name_score_;         ///< destination name of fitness score in signal object
    std::string name_convergence_;   ///< destination name of convergence results
    std::string name_transf_;        ///< desitnation name of final transformation

    bool has_conveged_; ///< flag whether icp converged

};

#endif // SEM_LAYERS_ICP_H_
