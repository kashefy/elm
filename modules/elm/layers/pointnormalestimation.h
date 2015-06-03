/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_POINTNORMALESTIMATION_H_
#define _ELM_LAYERS_POINTNORMALESTIMATION_H_

#ifndef __WITH_PCL
    // Disabling PointNormalEstimation layer due to no PCL support and defining it as a non-supported
    #include "elm/layers/layernotsupported.h"
namespace elm {
    class PointNormalEstimation : public ELM_LAYER_NOT_SUPPORTED(PointNormalEstimation, "Building with PCL is required for supporting PointNormalEstimation layer.");
} // namespace elm
#else   // __WITH_PCL

#include "elm/layers/layers_interim/base_singleinputfeaturelayer.h"

#include <boost/shared_ptr.hpp>

#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {

/**
 * @brief Layer for estimating normals in point cloud
 */
class PointNormalEstimation : public base_SingleInputFeatureLayer
{
public:
    // Parameters
    static const std::string PARAM_K_SEARCH; ///< maximum distance between connected points (maximum edge length)

    // I/O names
    static const std::string KEY_OUTPUT_POINT_CLOUD; ///< key to output vertices

    PointNormalEstimation();

    void Clear();

    void Reconfigure(const LayerConfig &cfg);

    void OutputNames(const LayerOutputNames &config);

    void Activate(const Signal &signal);

    void Response(Signal &signal);

protected:
    std::string name_dst_cloud_;    ///< desitnation name of output cloud

    CloudNrmlPtr dst_cloud_;

    int k_search_;
};

} // namespace elm

#endif // __WITH_PCL

#endif // _ELM_LAYERS_POINTNORMALESTIMATION_H_
